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


#ifndef BUTTONS_HPP_SUKJHKDJSHSDFJKSFJKJLKSFHGKFAJGFDF
#define BUTTONS_HPP_SUKJHKDJSHSDFJKSFJKJLKSFHGKFAJGFDF

#include <chrono>
#include <atomic>
#include <memory>
#include <poll.h>

#include <cppgpio/gpio.hpp>

namespace GPIO {

    /// The base class for "interrupt" based detection of changes on input lines. It
    /// actually does not use interrupts itself, but events signalled through poll()
    /// - therefore we do not have to care for timing and restarting of system calls.

    class InputDetect : public ObjectBase {
    public:
        typedef std::vector<unsigned int> gpiovec_t;
        
        /// start event detection on select gpio pins

        InputDetect(const gpiovec_t& gpios, GPIO_EDGE mode = GPIO_EDGE::BOTH);

        /// Properly destroy this class. This involves stopping the started event thread
        /// and closing all opened inputs.

        virtual ~InputDetect();

        /// After completion of class initialization (that is, registration of callables
        /// at the various function variables), signal here that the input detection
        /// thread could finally be run. If start() is never called, no detection will
        /// ever happen.

        bool start();

        /// it is also possible to stop the running thread, but you would normally not
        /// need to do it - the destructor does it automatically. If you call stop(), then
        /// be prepared that it could take up to 500ms for it to return (or even longer if
        /// the thread is currently not waiting for an event, but executing some long
        /// running custom task in your code).

        bool stop();

    protected:

        /// gets called by self when an input change is detected, and tells which gpio

        virtual void triggered(unsigned int gpio) {}

    private:
        typedef std::vector<pollfd> pollvec_t;
        pollvec_t m_pollvec;
        gpiovec_t m_gpiovec;
        bool m_terminate;
        std::unique_ptr<std::thread> m_waiter;

        /// the internal function that actually waits for status changes

        void event_loop();

    };

    /// The class DigitalIn models one input pin and its state, and performs digital debouncing of the input.
    /// It is used as class member in the various hardware modelling classes below.

    class DigitalIn {
    public:
        DigitalIn(/// GPIO pin to use
                  unsigned int pin,
                  /// pullup / down / none
                  GPIO_PULL pullupdown = GPIO_PULL::OFF,
                  /// define the interval that has to pass between two events
                  std::chrono::nanoseconds min_trigger_interval = std::chrono::nanoseconds(2 * 1000 * 1000),
                  /// define the minimum time the gpio pin has to be triggered
                  /// before a trigger is detected (to protect against noise on
                  /// the line)
                  std::chrono::nanoseconds min_hold_interval = std::chrono::nanoseconds(1 * 1000 * 1000));

        /// returns true if real state change (and no line noise or switch crackle)

        bool triggered() { return debounce(); }

        unsigned int get_pin() const { return m_pin; }
        unsigned int get_state() const { return m_state ? 1 : 0; }
        
    private:
        bool m_state;
        unsigned int m_pin;
        GPIO_PULL m_pullupdown;
        std::chrono::nanoseconds m_min_trigger_interval;
        std::chrono::nanoseconds m_min_hold_interval;
        std::chrono::steady_clock::time_point m_last_triggered;
        GPIOBase m_gpio;

        /// test against last triggered time, on pin

        bool debounce();

    };

    /// The class Switch models a physical switch, something that can be on or off at any given time. It offers
    /// function variables which will be called whenever the state changes. Register own functions to be called
    /// by simply assigning a lambda or a callable constructed with std::bind() to the function variables.
    
    class Switch : public InputDetect {
    public:
        Switch(/// GPIO pin to use
               unsigned int pin,
               /// pullup / down / none
               GPIO_PULL pullupdown = GPIO_PULL::OFF,
               /// define the interval that has to pass between two events
               std::chrono::nanoseconds min_trigger_interval = std::chrono::nanoseconds(2 * 1000 * 1000),  // 2 ms
               /// define the minimum time the gpio pin has to be triggered
               /// before a trigger is detected (to protect against noise on
               /// the line)
               std::chrono::nanoseconds min_hold_interval = std::chrono::nanoseconds(1 * 1000 * 1000))     // 1 ms
        : InputDetect({ pin }, GPIO_EDGE::BOTH)
        , m_pin(pin,
                pullupdown,
                min_trigger_interval,
                min_hold_interval)
        {}

        /// destructor

        virtual ~Switch() {}

        /// queries the state of the switch

        bool is_on() const { return m_pin.get_state(); }

        /// hooks for depending classes; get called when the button is switched.
        /// Register with a lambda expression or std::bind()
        /// see demo.cpp for examples

        std::function<void(bool)> f_switched {nullptr};
        std::function<void()> f_on {nullptr};
        std::function<void()> f_off {nullptr};
        
    protected:
        DigitalIn m_pin;

        virtual void triggered(unsigned int gpio);
    };
    
    
    /// The class DirectIn does not use a debouncer and is therefore optimized for maximum performance (it is in fact
    /// the Switch class with debouncer disabled). You could use it to act on events on inputs which are
    /// directly coupled to other digital outputs. The function variables of Switch are inherited for registering
    /// own callables.

    class DirectIn : public Switch {
    public:
        DirectIn(/// GPIO pin to use
                 unsigned int pin,
                 /// pullup / down / none
                 GPIO_PULL pullupdown = GPIO_PULL::OFF)
        : Switch(pin, pullupdown, std::chrono::nanoseconds(0), std::chrono::nanoseconds(0))
        {}

        virtual ~DirectIn() {}

    };
    
    /// The class Counter is a simple counter of the pulses on the GPIO pin. You can read the current count with get_count()
    /// and reset it with clear_count()

    class Counter : public Switch {
    public:
        Counter(/// GPIO pin to use
                unsigned int pin,
                /// pullup / down / none
                GPIO_PULL pullupdown = GPIO_PULL::OFF,
                /// define the interval that has to pass between two events
                std::chrono::nanoseconds min_trigger_interval = std::chrono::nanoseconds(2 * 1000 * 1000),  // 2 ms
                /// define the minimum time the gpio pin has to be triggered
                /// before a trigger is detected (to protect against noise on
                /// the line)
                std::chrono::nanoseconds min_hold_interval = std::chrono::nanoseconds(1 * 1000 * 1000));    // 1 ms

        virtual ~Counter() {}

        unsigned long get_count() const { return m_count; }
        void clear_count() { m_count = 0; }

    protected:
        std::atomic_ulong m_count;

    };
    
    /// This is a PushButton control class. It tells when a button is pushed, and how long it took
    /// to become released again. It offers function variables where you could register own callables
    /// which get called when a state change happens.

    class PushButton : public Switch {
    public:
        PushButton(/// GPIO pin to use
                   unsigned int pin,
                   /// pullup / down / none
                   GPIO_PULL pullupdown = GPIO_PULL::OFF,
                   /// define the interval that has to pass between two events
                   std::chrono::nanoseconds min_trigger_interval = std::chrono::nanoseconds(2 * 1000 * 1000),  // 2 ms
                   /// define the minimum time the gpio pin has to be triggered
                   /// before a trigger is detected (to protect against noise on
                   /// the line)
                   std::chrono::nanoseconds min_hold_interval = std::chrono::nanoseconds(1 * 1000 * 1000));    // 1 ms

        /// destructor

        virtual ~PushButton() {}
        
        /// hook for depending classes; gets called when the button is pushed
        /// Register with a lambda expression or std::bind()
        /// see demo.cpp for examples

        std::function<void()> f_pushed {nullptr};

        /// hook for depending classes; gets called when the button is released, value is time pushed in ns
        /// Register with a lambda expression or std::bind()
        /// see demo.cpp for examples

        std::function<void(std::chrono::nanoseconds)> f_released {nullptr};
        
    private:
        std::chrono::steady_clock::time_point last_pushed;

    };


    /// A rotary encoder class. Counts a value up/down with turning the dial, and permits setting a range.
    /// Also measures how fast the dial is turned. Register own callables at the function variables to be
    /// called on state changes.

    class RotaryDial : public InputDetect {
    public:
        RotaryDial(/// GPIO pin A to use
                   unsigned int pin_a,
                   /// GPIO pin B to use
                   unsigned int pin_b,
                   /// pullup / down / none
                   GPIO_PULL pullupdown = GPIO_PULL::OFF,
                   /// define the interval that has to pass between two events
                   std::chrono::nanoseconds min_trigger_interval = std::chrono::nanoseconds(2 * 1000 * 1000),  // 2 ms
                   /// define the minimum time the gpio pin has to be triggered
                   /// before a trigger is detected (to protect against noise on
                   /// the line)
                   std::chrono::nanoseconds min_hold_interval = std::chrono::nanoseconds(1 * 1000 * 1000),     // 1 ms
                   /// how many internal grey code steps mark one detent? 1, 2, and 4 are typical
                   int divider = 4);

        /// destructor

        virtual ~RotaryDial() {}

        void set_range(long min, long max);
        void set_value(long value) { m_value = value; }
        long get_value() const { return m_value; }
        unsigned int get_speed() const { return m_speed; }
        
        /// hooks for depending classes; get called when the dial is turned
        /// Register with a lambda expression or std::bind()
        /// see demo.cpp for examples

        std::function<void(bool, long)> f_dialed {nullptr};
        std::function<void()> f_up {nullptr};
        std::function<void()> f_down {nullptr};
        
    protected:
        void decode_dial();
        virtual void triggered(unsigned int gpio);

    private:
        DigitalIn m_pin_a, m_pin_b;
        std::atomic_long m_value {0};
        std::atomic_uint m_speed {0};
        long m_encvalue {0};
        int m_divider {4};
        long m_min {std::numeric_limits<long>::min()};
        long m_max {std::numeric_limits<long>::max()};
        bool m_use_range {false};
        unsigned int m_lastseq {0};
        std::chrono::steady_clock::time_point m_last_triggered;
        std::chrono::steady_clock::time_point m_last_dialed;

        void set_speed(std::chrono::nanoseconds timediff);

    };
    
 
}

#endif // #ifndef BUTTONS_HPP_SUKJHKDJSHSDFJKSFJKJLKSFHGKFAJGFDF
