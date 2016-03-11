//
// buttons.cpp
//
//  Copyright © 2016 Joachim Schurig. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice, this
//  list of conditions and the following disclaimer.
//  2. Redistributions in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
//  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <poll.h>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ratio>
#include <limits>
#include <iostream>


#include <cppgpio/buttons.hpp>
#include "tools.hpp"

#if __cplusplus <= 201103L
// helper if the current compiler only supports C++11
#include "make_unique.hpp"
#endif


using namespace GPIO;


void InputDetect::event_loop()
{
    // clear pending events

    for (auto& it : m_pollvec) {

        int count;
        ::ioctl(it.fd, FIONREAD, &count);
        for (int i = 0; i < count; ++i) {
            char ch;
            ::read(it.fd, &ch, 1);
        }
        
    }

    // start the event loop

    for (;;) {

        // poll for half a second, then check if we shall terminate

        int count = ::poll(m_pollvec.data(), static_cast<unsigned int>(m_pollvec.size()), 500);
        
        // shall we end execution?

        if (m_terminate) {
            for (auto& it : m_pollvec) {
                ::close(it.fd);
            }
            return;
        }

        // no fds got a notification?

        if (!count) continue;
        
        gpiovec_t::size_type idx = 0;

        for (auto& it : m_pollvec) {

            // check if this fd got a notification

            if (it.revents != 0) {
                
                // clear the event - read a byte and seek back to start

                unsigned char ch;
                ::read(it.fd, &ch, 1);
                ::lseek(it.fd, 0, SEEK_SET);
                
                // and call the virtual trigger with the detected pin

                triggered(m_gpiovec[idx]);

                // if count is 0 we have no more signalled events

                if (!--count) break;
                
            }

            ++idx;
        }
    }
}


InputDetect::InputDetect(const gpiovec_t& gpios, GPIO_EDGE mode)
: m_gpiovec(gpios)
, m_terminate(false)
{
    if (GPIOBase::simulation()) return;
    if (m_gpiovec.empty()) return;
    
    // open export interface - it may be reused multiple times in the loop below

    Tools::AutoFile fexp("/sys/class/gpio/export", O_WRONLY);

    // iterate over the list of gpio pins to use

    for (auto i : m_gpiovec) {
        
        // export the pin
        std::string pin_s = std::to_string(i) + '\n';
        try {
            fexp.write(pin_s);
        } catch (Tools::ToolsError& err) {

            // rethrow if errno is not EBUSY (which happens when the pin was already exported
            // and forgotten to get unexported later, e.g. through a program crash)

            if (errno != EBUSY) throw;
        }

        // construct the fixed prefix for the other operations (file name writes)

        std::string prefix = std::string("/sys/class/gpio/gpio") + std::to_string(i);

        Tools::AutoFile f;

        std::string name = prefix + "/direction";
        for (int x = 0; x < 10; ++x) {
            if (f.open_nothrow(name, O_WRONLY)) break;

            // sleep for a while - the pin first needs to be exported..

            std::this_thread::sleep_for(std::chrono::microseconds(10*1000));
        }
        if (*f < 0) throw GPIOError(name + ": cannot open GPIO direction interface ");
        f.write("in\n");
        f.close();
        
        name = prefix + "/edge";
        f.open(name, O_WRONLY);

        switch (mode) {
            case GPIO_EDGE::FALLING:
                f.write("falling\n");
                break;
                
            case GPIO_EDGE::RISING:
                f.write("rising\n");
                break;
                
            case GPIO_EDGE::BOTH:
                f.write("both\n");
                break;
                
            default:
                break;
        }
        
        f.close();
        
        name = prefix + "/value";
        f.open(name, O_RDWR);
        
        m_pollvec.push_back( { f.release(), POLLPRI, 0 } );
        
    }

    // close the export interface

    fexp.close();
}

bool InputDetect::start()
{
    // already running the interrupt thread?

    if (m_waiter != nullptr) return false;

    // no input defined?

    if (m_pollvec.empty()) return false;

    // no - start the thread now

    m_terminate = false;

    m_waiter = std::make_unique<std::thread>(&InputDetect::event_loop, this);

    return true;
}

bool InputDetect::stop()
{
    if (!m_waiter) return false;

    // we do not need to use atomic access for m_terminate

    m_terminate = true;

    // we have to wait for the polling thread to terminate - otherwise
    // it might call into a destructed class..

    m_waiter->join();

    // delete the thread object

    m_waiter.reset(nullptr);

    return true;
}

InputDetect::~InputDetect()
{
    // stop the thread if still running

    stop();

    if (GPIOBase::simulation()) return;
    if (m_gpiovec.empty()) return;

    // open unexport interface

    Tools::AutoFile funexp("/sys/class/gpio/unexport", O_WRONLY);

    // finally unexport all outputs again
    // iterate over the list of gpio pins to use

    for (auto i : m_gpiovec) {

        // unexport the pin
        std::string pin_s = std::to_string(i) + '\n';
        funexp.write(pin_s);

    }
}




/// Simple debouncer, allowing only one event per defined nanosecond interval.
/// To prevent triggering by short noise pulses checks for minimum trigger hold.

bool DigitalIn::debounce()
{
    // check if the input has the expected state

    if (m_gpio.read(m_pin) != ((m_pullupdown != GPIO_PULL::UP) ? !m_state : m_state)) return false;
    
    // is this a truly perfect digital 0 1 input? Then do not debounce..

    if (m_min_trigger_interval.count() == 0) return true;
    
    auto now = std::chrono::steady_clock::now();
    if ((now - m_last_triggered) < m_min_trigger_interval) return false;

    if (m_min_hold_interval.count() > 0) {

        // sleep a wink

        std::this_thread::sleep_for(m_min_hold_interval);
    
        // and check if the pin is still triggered (pin is 0 if pulldown and switch closed, otherwise 1

        if (m_gpio.read(m_pin) != ((m_pullupdown != GPIO_PULL::UP) ? !m_state : m_state)) return false;
    }
    
    // finally assign this event as the last trigger

    m_last_triggered = now;
    
    // toggle state

    m_state = !m_state;
    
    // and return success

    return true;
}


DigitalIn::DigitalIn(// GPIO pin to use
                     unsigned int pin,
                     // pullup / down / none
                     GPIO_PULL pullupdown,
                     // define the interval that has to pass between two events
                     std::chrono::nanoseconds min_trigger_interval,
                     // define the minimum time the gpio pin has to be triggered
                     // before a trigger is detected (to protect against noise on
                     // the line)
                     std::chrono::nanoseconds min_hold_interval)
: m_state(false)
, m_pin(pin)
, m_pullupdown(pullupdown)
, m_min_trigger_interval(min_trigger_interval)
, m_min_hold_interval(min_hold_interval)
{
    // switch the pin to input mode

    m_gpio.mode(m_pin, GPIO_MODE::GPIO_INPUT);

    // and set the requested pullup/down mode

    m_gpio.pullupdown(m_pin, m_pullupdown);

    // set the right initial states (the pin could have been
    // active during initialisation)

    m_state = m_gpio.read(m_pin);

    // invert logic if PULL_UP active

    if (m_pullupdown == GPIO_PULL::UP) m_state = !m_state;
}



/// the interrupt handler for the Switch

void Switch::triggered(unsigned int gpio)
{
    // check if this is really a new event, or still a bouncing switch

    if (!m_pin.triggered()) return;
    
    // now call the virtual functions to trigger some action

    switch (m_pin.get_state()) {
        case false:
            if (f_off) f_off();
            break;

        case true:
            if (f_on) f_on();
            break;
    }
    
    if (f_switched) f_switched(m_pin.get_state());
}


Counter::Counter(unsigned int pin,
                 GPIO_PULL pullupdown,
                 std::chrono::nanoseconds min_trigger_interval,
                 std::chrono::nanoseconds min_hold_interval)
: Switch(pin,
         pullupdown,
         min_trigger_interval,
         min_hold_interval)
, m_count(0)
{

    // set Switch::f_on with a lambda expression to increase our counter

    f_on = [&]() { ++m_count; };
}


PushButton::PushButton(unsigned int pin,
                       GPIO_PULL pullupdown,
                       std::chrono::nanoseconds min_trigger_interval,
                       std::chrono::nanoseconds min_hold_interval)
: Switch(pin,
         pullupdown,
         min_trigger_interval,
         min_hold_interval)
{
    // set Switch::f_on with a lambda expression to forward the push event

    f_on = [&]() {
        last_pushed = std::chrono::steady_clock::now();
        if (f_pushed) f_pushed();
    };
    
    // set Switch::f_off with a lambda expression to forward the released event

    f_off = [&]() {
        auto now = std::chrono::steady_clock::now();
        if (f_released) f_released(now - last_pushed);
    };
}



/// generate a speed value from timediff,
/// by counting the changes per second (with 1 being the minimum)

void RotaryDial::set_speed(std::chrono::nanoseconds timediff)
{
    if (timediff.count() > 1000 * 1000 * 1000) m_speed = 1;
    else m_speed = 1000 * 1000 * 1000 / timediff.count();
}

void RotaryDial::set_range(long min, long max)
{
    if (min > max) throw GPIOError("min > max");
    
    m_min = min;
    m_max = max;
    
    if (m_min > m_value) m_value = m_min;
    else if (m_max < m_value) m_value = m_max;
    
    m_use_range = true;
}

void RotaryDial::decode_dial()
{
    // Get the current state, use a ^ b | b << 1 to generate a 2 bit sequence by turning the dial.
    // For the logic behind the next two lines see:
    // http://guy.carpenter.id.au/gaugette/2013/01/14/rotary-encoder-library-for-the-raspberry-pi/
    
    unsigned int seq   = (m_pin_a.get_state() ^ m_pin_b.get_state()) | m_pin_b.get_state() << 1;
    unsigned int delta = (seq - m_lastseq) & 0b11;
    
    //    delta	meaning
    //    0	no change
    //    1	1 step clockwise
    //    2	2 steps clockwise or counter-clockwise
    //    3	1 step counter clockwise
    
    // store value for next turn
    m_lastseq = seq;
    
    auto now = std::chrono::steady_clock::now();
    if (m_divider > 1 && m_encvalue && now - m_last_triggered > std::chrono::nanoseconds(500 * 1000 * 1000)) {
        // reset m_encvalue, it has run out of sync by some bounces, and
        // needs certainly less than half a second between two events in a grey sequence
        m_encvalue = 0;
    }
    m_last_triggered = now;
    
    switch (delta) {
        case 0:
            // no turn
            return;
        case 1:
            ++m_encvalue;
            break;
        case 2:
            // we could assume a fast turn to the last direction, but we simply
            // drop this result (actually, the interrupt driven approach on the
            // Raspberry is fast enough to never run into this case)
            return;
        case 3:
            --m_encvalue;
            break;
    }

    if (std::abs(m_encvalue) >= m_divider) {

        set_speed(now - m_last_dialed);
        
        if (m_encvalue > 0) {
            m_encvalue = 0;
            if (m_use_range && m_value == m_max) return;
            ++m_value;
            // call the function pointer
            if (f_up) f_up();
        } else {
            m_encvalue = 0;
            if (m_use_range && m_value == m_min) return;
            --m_value;
            // call the function pointer
            if (f_down) f_down();
        }
        
        // call the function pointer
        if (f_dialed) f_dialed(delta == 1, m_value);

        m_last_dialed = now;
    }
    
}

void RotaryDial::triggered(unsigned int gpio)
{
    // do proper debouncing and state changes on the pins

    if (gpio == m_pin_a.get_pin()) {
        if (!m_pin_a.triggered()) return;
    } else if (gpio == m_pin_b.get_pin()) {
        if (!m_pin_b.triggered()) return;
    }

    decode_dial();
}

RotaryDial::RotaryDial(unsigned int pin_a,
                       unsigned int pin_b,
                       GPIO_PULL pullupdown,
                       std::chrono::nanoseconds min_trigger_interval,
                       std::chrono::nanoseconds min_hold_interval,
                       int divider)
: InputDetect({ pin_a, pin_b },
              GPIO_EDGE::BOTH)
, m_pin_a(pin_a,
          pullupdown,
          min_trigger_interval,
          min_hold_interval)
, m_pin_b(pin_b,
          pullupdown,
          min_trigger_interval,
          min_hold_interval)
, m_divider(divider)
{
    // the code would even run with divider == 0 (which would simply be interpreted as 1),
    // therefore do not throw on that condition
    
    // init to the right state - we do not know how the rotary encoder has set its outputs
    // at start

    m_lastseq = (m_pin_a.get_state() ^ m_pin_b.get_state()) | m_pin_b.get_state() << 1;
}


