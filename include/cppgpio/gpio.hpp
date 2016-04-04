//
//  gpio.hpp
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


#ifndef GPIO_H_ASZJAHSGCHCLNISKUGDHSILJDSV
#define GPIO_H_ASZJAHSGCHCLNISKUGDHSILJDSV

#include <exception>
#include <thread>
#include <functional>
#include <vector>
#include <string>
#include <map>
#include <stdexcept>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <typeinfo>
#include <memory>
#include <mutex>


namespace GPIO {
    
    class GPIOError : public std::runtime_error {
        using std::runtime_error::runtime_error;
    };
    typedef std::mutex Mutex;
    typedef std::lock_guard<Mutex> Lock;
    typedef std::recursive_mutex RecursiveMutex;
    typedef std::lock_guard<RecursiveMutex> RecursiveLock;

    /// base class for all GPIO objects (to allow common storage and safe destruction
    /// from the base object via virtual destructor)
    
    class ObjectBase {
    public:
        ObjectBase() = default;
        virtual ~ObjectBase() = 0;
    };

    typedef std::unique_ptr<ObjectBase> UniqueObjectBase;
    typedef std::shared_ptr<ObjectBase> SharedObjectBase;

    
    /// This is the implementation of a concrete GPIO control, the one in the BCM2835.
    /// This is a temporary solution which will get replaced by a factory model
    /// to choose from different GPIOs, with the below public functionality being
    /// the factory abstract base.
    
    /// See peripherals documentation of BCM2835 at
    /// https://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf

    enum class GPIO_MODE : uint8_t {
        GPIO_INPUT          = 0,
        GPIO_OUTPUT         = 1,
        PWM_OUTPUT          = 2,
        SOFT_PWM_OUTPUT     = 3,
        PWM_TONE_OUTPUT     = 4,
        SOFT_TONE_OUTPUT    = 5
    };
    
    // this "happen" to be the real values needed for the bcm2835, but for other
    // chips they might need translation

    enum class GPIO_PULL : uint8_t {
        OFF                 = 0,
        DOWN                = 1,
        UP                  = 2
    };
    
    // this "happen" to be the real values needed for the bcm2835, but for other
    // chips they might need translation
    
    enum class GPIO_EDGE : uint8_t {
        FALLING             = 0,
        RISING              = 1,
        BOTH                = 2
    };
    
    enum class PWM_MODE : uint8_t {
        MARKSPACE           = 0,
        BALANCED            = 1,
        INVALID             = 2
    };

    /// This class has no non-static member variables, but its public functions
    /// are nonetheless declared non-static, to force initialization of the underlying
    /// memory map through the default constructor.
    
    class GPIOBase {
    public:
        GPIOBase();

        /// Set "simulation" mode - that is, no real IO, but instead dummy operating
        /// only. Simulation mode is set automatically when this code is run on OS
        /// other than Linux, but if you want to run in this mode on Linux you
        /// have to call simulation(true) explicitly.

        static void simulation(bool yes);

        /// returns true if simulation (dummy) mode is set

        static bool simulation() { return simulate; }

        /// Call before any instantiation of a GPIOBase object if you want to
        /// force full register access via IO mapping even when /dev/gpiomem is
        /// available. When you switch into this mode and are not running as root,
        /// the first instantiation of a GPIOBase object will throw an exception.
        /// When you are root, you gain access on all registers of the bcm2835, not
        /// only on I/O pins, but with the current implementation of this library the
        /// only advantage is that you can then set the PWM clock frequency and use
        /// hardware PWM on two outputs.

        static bool force_full_mapping();

        /// sets the I/O mode of a gpio

        void mode(unsigned int gpio, GPIO_MODE mode) const;

        /// sets pullup/down for a gpio

        void pullupdown(unsigned int gpio, GPIO_PULL pud) const;

        /// read status of an input

        bool read(unsigned int gpio) const
        {
            return int_read(gpio);
        }
        
        /// clear (set to 0V) an output

        void clear(unsigned int gpio) const
        {
            int_clear(gpio);
        }
        
        /// set (set to 3.3V) an output

        void set(unsigned int gpio) const
        {
            int_set(gpio);
        }

        /// if value is 0 the output will be cleared, else will be set

        void write(unsigned int gpio, bool value) const
        {
            (value) ? set(gpio) : clear(gpio);
        }

        /// returns true if this GPIO can be run in hardware PWM mode

        bool has_hard_pwm(unsigned int gpio) const;

        /// set the on-off ratio for the hardware PWM output

        void pwm_write(unsigned int gpio, unsigned int value) const;

        /// set the mode (mark/space or balanced) for the hardware PWM output

        void pwm_mode(unsigned int gpio, PWM_MODE mode) const;

        /// set the value range for the hardware PWM output

        void pwm_range(unsigned int gpio, unsigned int range) const;

        /// set the clock divider for the hardware PWM output

        void pwm_clock(unsigned int gpio, unsigned int divisor) const;

        /// Set the value range for the soft PWM output (the soft PWM is
        /// generated by a looping thread, not by PWM hardware - it is
        /// less accurate, as the thread will be scheduled in and out
        /// of execution, but it can be run on any output, and it does
        /// not need root priviledges to be run).

        void soft_pwm_range(unsigned int gpio, unsigned int range) const;

        /// Set the on-off ratio for the soft PWM output (the soft PWM is
        /// generated by a looping thread, not by PWM hardware - it is
        /// less accurate, as the thread will be scheduled in and out
        /// of execution, but it can be run on any output, and it does
        /// not need root priviledges to be run).

        void soft_pwm_write(unsigned int gpio, unsigned int value) const;

        /// set the frequency for the hardware PWM tone (square wave) output

        void pwm_tone_write(unsigned int gpio, unsigned int frequency) const;

        /// set the frequency for the soft PWM tone (square wave) output
        /// (range is from 0..10000 Hz)

        void soft_tone_write(unsigned int gpio, unsigned int frequency) const;

    private:
        enum class PWM_CHANNEL : std::uint8_t {
            CH0         = 0,
            CH1         = 1
        };

        enum {
            PWM_CHANNELS    = 2
        };

        enum class FSEL : uint8_t {
            INPUT       = 0b000,
            OUTPUT      = 0b001,
            ALT0        = 0b100,
            ALT1        = 0b101,
            ALT2        = 0b110,
            ALT3        = 0b111,
            ALT4        = 0b011,
            ALT5        = 0b010
        };

        static std::once_flag initlock;
        static void init();

        struct bcm2835_peripheral_t {
            unsigned long addr_p;
            int mem_fd;
            void *map;
            volatile unsigned int *addr;
        };

        static bool simulate;
        static bool want_full_mapping;
        static bool gpio_only;
        static unsigned long peripherals_base;
        static unsigned long peripherals_size;
        static std::unique_ptr<unsigned long[]> peripherals_sim;
        static volatile unsigned long* peripherals;
        static volatile unsigned long* gpio_addr;
        static volatile unsigned long* pwm_addr;
        static volatile unsigned long* clk_addr;
        static volatile unsigned long* pads_addr;
        static volatile unsigned long* spi0_addr;
        static volatile unsigned long* bsc0_addr;
        static volatile unsigned long* bsc1_addr;
        static volatile unsigned long* st_addr;

        static Mutex pullupdown_mutex;

        class SoftThread {
        private:
            std::unique_ptr<std::thread> m_thread;
            unsigned int m_range;
            unsigned int m_value;
            std::atomic_uint m_mark;
            std::atomic_uint m_space;
            unsigned int m_gpio;

            void run_tone();
            void run_pwm();

        public:
            void set_range(unsigned int range);
            void set_value(unsigned int value);
            void terminate() { set_range(0); }
            SoftThread(GPIO_MODE what       = GPIO_MODE::SOFT_PWM_OUTPUT,
                       unsigned int gpio    = 0,
                       unsigned int range   = 1024,
                       unsigned int value   = 0);
            ~SoftThread();
        };

        typedef std::unique_ptr<SoftThread> UniqueSoftThread;
        typedef std::map<unsigned int, UniqueSoftThread> soft_thread_map_t;
        static soft_thread_map_t soft_thread_map;
        static Mutex m_soft_thread_mutex;

        struct pwm_t {
            // converting helper constructor for init
            pwm_t(unsigned int fsel_p, unsigned int channel_p)
            : fsel(static_cast<FSEL>(fsel_p))
            , channel(static_cast<PWM_CHANNEL>(channel_p)) {}
            pwm_t(FSEL fsel_p, PWM_CHANNEL channel_p)
            : fsel(fsel_p)
            , channel(channel_p) {}

            FSEL            fsel;
            PWM_CHANNEL     channel;
        };
        typedef std::vector<pwm_t> pwm_vec_t;
        static pwm_vec_t pwm_reg;
        static unsigned int pwm_range_val[PWM_CHANNELS];
        static PWM_MODE pwm_mode_val[PWM_CHANNELS];
        static unsigned int pwm_clock_val; // one clock for all channels..

        static void int_set_pwm_mode(PWM_CHANNEL channel, PWM_MODE mode);
        static void int_set_pwm_range(PWM_CHANNEL channel, unsigned int range);
        static void int_set_pwm_clock(PWM_CHANNEL channel, unsigned int divisor);
        static void int_pwm_write(PWM_CHANNEL channel, unsigned int value);

        static void int_set_mode(unsigned int gpio, FSEL mode);

        static inline void int_set(unsigned int gpio)
        {
            *(gpio_addr + 7 + gpio/32) = 1 << (gpio%32);
        }

        static inline void int_clear(unsigned int gpio)
        {
            *(gpio_addr + 10 + gpio/32) = 1 << (gpio%32);
        }

        static inline bool int_read(unsigned int gpio)
        {
            return *(gpio_addr + 13 + gpio/32) &= (1 << (gpio%32));
        }

        void check_hard_pwm(unsigned int gpio) const;

    };

}


#endif