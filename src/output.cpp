//
//  output.cpp
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

#include <cppgpio/output.hpp>

using namespace GPIO;


DigitalOut::DigitalOut(unsigned int pin)
: m_pin(pin)
{
    m_gpio.mode(m_pin, GPIO_MODE::GPIO_OUTPUT);
}

DigitalOut::~DigitalOut()
{
    m_gpio.mode(m_pin, GPIO_MODE::GPIO_INPUT);
}

void DigitalOut::on(std::chrono::nanoseconds nanos) const
{
    on();
    std::this_thread::sleep_for(nanos);
    off();
}

void DigitalOut::off(std::chrono::nanoseconds nanos) const
{
    off();
    std::this_thread::sleep_for(nanos);
    on();
}


ShiftOut::ShiftOut(unsigned int pin_data, unsigned int pin_clock, unsigned int pin_latch, DIRECTION direction)
: m_pin_data(pin_data)
, m_pin_clock(pin_clock)
, m_pin_latch(pin_latch)
, m_direction(direction)
{
    if (   m_pin_data  == m_pin_latch
        || m_pin_data  == m_pin_clock
        || m_pin_latch == m_pin_clock) {
        throw GPIOError("need 3 different GPIO pins for ShiftOut configuration");
    }
    m_gpio.mode(m_pin_data,  GPIO_MODE::GPIO_OUTPUT);
    m_gpio.mode(m_pin_clock, GPIO_MODE::GPIO_OUTPUT);
    m_gpio.mode(m_pin_latch, GPIO_MODE::GPIO_OUTPUT);
    m_gpio.clear(m_pin_data);
    m_gpio.clear(m_pin_clock);
    m_gpio.clear(m_pin_latch);
}

ShiftOut::~ShiftOut()
{
    m_gpio.mode(m_pin_data,  GPIO_MODE::GPIO_INPUT);
    m_gpio.mode(m_pin_clock, GPIO_MODE::GPIO_INPUT);
    m_gpio.mode(m_pin_latch, GPIO_MODE::GPIO_INPUT);
}

void ShiftOut::write1(bool value) const
{
    m_gpio.write(m_pin_data, value);
    m_gpio.write(m_pin_clock, true);
    std::this_thread::sleep_for(std::chrono::nanoseconds(10*1000));
    m_gpio.write(m_pin_clock, false);
    std::this_thread::sleep_for(std::chrono::nanoseconds(10*1000));
}


PWMOut::PWMOut(unsigned int pin, unsigned int range, unsigned int ratio)
: m_pin(pin)
{
    m_is_soft_pwm = !m_gpio.has_hard_pwm(m_pin);
    m_gpio.mode(m_pin, (m_is_soft_pwm) ? GPIO_MODE::SOFT_PWM_OUTPUT : GPIO_MODE::PWM_OUTPUT);
    set_range(range);
    set_ratio(ratio);
}

PWMOut::~PWMOut()
{
    m_gpio.mode(m_pin, GPIO_MODE::GPIO_INPUT);
}

void PWMOut::set_range(unsigned int range) const
{
    m_is_soft_pwm ? m_gpio.soft_pwm_range(m_pin, range) : m_gpio.pwm_range(m_pin, range);
}

void PWMOut::set_ratio(unsigned int ratio) const
{
    m_is_soft_pwm ? m_gpio.soft_pwm_write(m_pin, ratio) : m_gpio.pwm_write(m_pin, ratio);
}

void ToneOut::set_freq(unsigned int hz) const
{
    if (m_is_soft_tone) m_gpio.soft_tone_write(m_pin, hz);
    else m_gpio.pwm_tone_write(m_pin, hz);
}

ToneOut::ToneOut(unsigned int pin, unsigned int hz)
: m_pin(pin)
{
    m_is_soft_tone = !m_gpio.has_hard_pwm(m_pin);
    m_gpio.mode(m_pin, (m_is_soft_tone) ? GPIO_MODE::SOFT_TONE_OUTPUT : GPIO_MODE::PWM_TONE_OUTPUT);
    set_freq(hz);
}

ToneOut::~ToneOut()
{
    m_gpio.mode(m_pin, GPIO_MODE::GPIO_INPUT);
}


