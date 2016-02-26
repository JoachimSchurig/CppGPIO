//
//  output.hpp
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

#ifndef output_hpp_SILJGLSDKHGZJSDVJSDVHBJSDZVBSDKVBHKSDHBV
#define output_hpp_SILJGLSDKHGZJSDVJSDVHBJSDZVBSDKVBHKSDHBV

#include <chrono>
#include <cppgpio/gpio.hpp>

namespace GPIO {

    /// The class DigitalOut is controlling one single output pin, which
    /// can be switched on or off, or switched on or off for a specified
    /// period of time.

    class DigitalOut : public ObjectBase {
    public:

        /// Constructor for the DigitalOut object. Switches IO pin to output.
        /// Requires the GPIO pin as argument.

        DigitalOut(unsigned int pin);

        /// Destructor switches IO pin back to input mode.

        virtual ~DigitalOut();

        /// Switch output on

        void  on() const { m_gpio.set(m_pin);   }

        /// Switch output off

        void off() const { m_gpio.clear(m_pin); }

        /// Switch output on for a given intervall, then switch it off again

        void  on(std::chrono::nanoseconds nanos) const;

        /// Switch output off for a given intervall, then switch it on again
        
        void off(std::chrono::nanoseconds nanos) const;
        
    private:
        unsigned int m_pin;
        GPIOBase m_gpio;
    };


    /// The class ShiftOut permits to serially transmit any scalar type on
    /// an output port. It also needs a clock and latch port to signal the
    /// validity of each sent bit as well as the end of transmission. This
    /// permits to extend the output ports by simple 74xxx shift registers.

    class ShiftOut : public ObjectBase {
    public:
        enum class DIRECTION : uint8_t { LSBFIRST, MSBFIRST };

        /// Constructor for ShiftOut. Needs three IO ports which
        /// will be switched into output mode. Direction parameter
        /// indicates if the values shall be transmitted with LSB
        /// or MSB first.

        ShiftOut(unsigned int pin_data,
                 unsigned int pin_clock,
                 unsigned int pin_latch,
                 DIRECTION direction = DIRECTION::LSBFIRST);

        /// The destructor switches the mode of the ports back to
        /// input mode.

        virtual ~ShiftOut();

        /// Write a value of any scalar type to the selected port.
        /// Throws if the value is not a scalar type.
        /// The bits parameter determines how many bits of value
        /// are written. When bits is larger than the count of bits
        /// of value's type definition, a GPIOError is thrown.

        template <class VALUE>
        void write(const VALUE& value, uint8_t bits) const
        {
            static_assert(std::is_scalar<VALUE>::value, "need scalar type");
            static_assert(bits > sizeof(value)*8, "bits exceed value type's size");
            
            latch(false);
            VALUE mask;
            if (m_direction == DIRECTION::LSBFIRST) mask = 1;
            else mask = 1 << (bits - 1);
            for (uint8_t b = 0; b < bits; ++b){
                write1(value & mask);
                if (m_direction == DIRECTION::LSBFIRST) mask <<= 1;
                else mask >>= 1;
            }
            latch(true);
        }

        /// Write a value of any scalar type to the selected port.
        /// Throws if the value is not a scalar type.
        /// Determines bit size by looking at the type definition
        /// of the value.

        template <class VALUE>
        void write(const VALUE& value) const
        {
            write(value, sizeof(VALUE)*8);
        }

    protected:
        void latch(bool trigger) const { m_gpio.write(m_pin_latch, trigger); }
        void write1(bool value) const;

    private:
        unsigned int m_pin_data;
        unsigned int m_pin_clock;
        unsigned int m_pin_latch;
        DIRECTION m_direction;
        GPIOBase m_gpio;
    };

    /// The class PWMOut controls a PWM output port. If the selected port does not
    /// support hardware PWM output, the software PWM emulation is used (which is
    /// less accurate due to kernel scheduling).

    class PWMOut : public ObjectBase {
    public:

        /// Constructor for PWMOut. Requires an output port and optionally a range
        /// and a ratio value for the initial PWM output configuration. Per default
        /// range will be set to 1024 and ratio to 0.

        PWMOut(unsigned int pin, unsigned int range = 1024, unsigned int ratio = 0);

        /// Destructor for PWMOut. Switches the used port back into input mode.

        virtual ~PWMOut();

        /// Set the range to a new value. If range > ratio the behaviour is undefined.

        void set_range(unsigned int range) const;

        /// Set the ratio to a new value. If range > ratio the behaviour is undefined.
        
        void set_ratio(unsigned int ratio) const;

        /// Returns true if this is not a hardware PWM channel but a software emulation.

        bool is_software_emulation() const { return m_is_soft_pwm; }
        
    private:
        unsigned int m_pin;
        bool m_is_soft_pwm;
        GPIOBase m_gpio;
    };

    /// The class ToneOut permits to output square waves of a given frequency at any
    /// output port. If available, a hard PWM channel will be used, but a software
    /// emulation with limited frequency range (0..10000 Hz) is available on any port.

    class ToneOut : public ObjectBase {
    public:

        /// Constructor for ToneOut. Requires an output port and optionally a frequency.
        /// Per default, frequency will be set to 0 Hz.

        ToneOut(unsigned int pin, unsigned int hz = 0);

        /// Destructor for ToneOut. Switches the used port back into input mode.
        
        virtual ~ToneOut();

        /// Set the frequency to a new value. If this is a software emulation the function
        /// throws if the frequency is > 10000 Hz.

        void set_freq(unsigned int hz) const;

        /// Returns true if this is not a hardware PWM channel but a software emulation.

        bool is_software_emulation() const { return m_is_soft_tone; }

    private:
        unsigned int m_pin;
        bool m_is_soft_tone;
        GPIOBase m_gpio;
    };

    /// BlinkOut is simply a ToneOut (but typically run with frequencies in the range of 1-20 Hz)
    
    typedef ToneOut BlinkOut;

}

#endif /* output_hpp */
