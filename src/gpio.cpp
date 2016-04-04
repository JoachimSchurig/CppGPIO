//
//  gpio.cpp
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

#include <sys/mman.h>
#include <vector>
#include <memory>

#include <cppgpio/gpio.hpp>
#include "tools.hpp"

#if __cplusplus <= 201103L
// helper if the current compiler only supports C++11
#include "make_unique.hpp"
#endif



using namespace GPIO;

ObjectBase::~ObjectBase() = default;

std::once_flag GPIOBase::initlock;

unsigned long GPIOBase::peripherals_base = 0;
unsigned long GPIOBase::peripherals_size = 0;

bool GPIOBase::simulate = false;
bool GPIOBase::want_full_mapping = false;
bool GPIOBase::gpio_only = false;

std::unique_ptr<unsigned long[]> GPIOBase::peripherals_sim;
volatile unsigned long* GPIOBase::peripherals;
volatile unsigned long* GPIOBase::gpio_addr;
volatile unsigned long* GPIOBase::pwm_addr;
volatile unsigned long* GPIOBase::clk_addr;
volatile unsigned long* GPIOBase::pads_addr;
volatile unsigned long* GPIOBase::spi0_addr;
volatile unsigned long* GPIOBase::bsc0_addr;
volatile unsigned long* GPIOBase::bsc1_addr;
volatile unsigned long* GPIOBase::st_addr;
Mutex GPIOBase::pullupdown_mutex;

enum {
    GPIO_MEM_BLOCK_SIZE = 4 * 1024
};

enum REG_BASE {
    ST_BASE         = 0x3000,
    GPIO_PADS       = 0x100000,
    CLOCK_BASE      = 0x101000,
    GPIO_BASE       = 0x200000,
    SPI0_BASE       = 0x204000,
    BSC0_BASE       = 0x205000,
    GPIO_PWM        = 0x20C000,
    BSC1_BASE       = 0x804000
};

enum PWM_CONTROL_REGION {
    PWM_CONTROL     = 0,
    PWM_STATUS      = 1,
    PWM0_RANGE      = 4,
    PWM0_DATA       = 5,
    PWM1_RANGE      = 8,
    PWM1_DATA       = 9
};

enum PWM_INT_MODE {
    PWM0_MS_MODE    = 0x0080, // Run in MS mode
    PWM0_USEFIFO    = 0x0020, // Data from FIFO
    PWM0_REVPOLAR   = 0x0010, // Reverse polarity
    PWM0_OFFSTATE   = 0x0008, // Ouput Off state
    PWM0_REPEATFF   = 0x0004, // Repeat last value if FIFO empty
    PWM0_SERIAL     = 0x0002, // Run in serial mode
    PWM0_ENABLE     = 0x0001, // Channel Enable
    PWM1_MS_MODE    = 0x8000, // Run in MS mode
    PWM1_USEFIFO    = 0x2000, // Data from FIFO
    PWM1_REVPOLAR   = 0x1000, // Reverse polarity
    PWM1_OFFSTATE   = 0x0800, // Ouput Off state
    PWM1_REPEATFF   = 0x0400, // Repeat last value if FIFO empty
    PWM1_SERIAL     = 0x0200, // Run in serial mode
    PWM1_ENABLE     = 0x0100  // Channel Enable
};

enum PWM_CLOCK_REG {
    PWMCLK_CNTL     = 40,
    PWMCLK_DIV      = 41
};

enum GPIO_REG_OFFSETS {
    GPFSEL0         = 0x0000, // GPIO Function Select 0
    GPFSEL1         = 0x0004, // GPIO Function Select 1
    GPFSEL2         = 0x0008, // GPIO Function Select 2
    GPFSEL3         = 0x000c, // GPIO Function Select 3
    GPFSEL4         = 0x0010, // GPIO Function Select 4
    GPFSEL5         = 0x0014, // GPIO Function Select 5
    GPSET0          = 0x001c, // GPIO Pin Output Set 0
    GPSET1          = 0x0020, // GPIO Pin Output Set 1
    GPCLR0          = 0x0028, // GPIO Pin Output Clear 0
    GPCLR1          = 0x002c, // GPIO Pin Output Clear 1
    GPLEV0          = 0x0034, // GPIO Pin Level 0
    GPLEV1          = 0x0038, // GPIO Pin Level 1
    GPEDS0          = 0x0040, // GPIO Pin Event Detect Status 0
    GPEDS1          = 0x0044, // GPIO Pin Event Detect Status 1
    GPREN0          = 0x004c, // GPIO Pin Rising Edge Detect Enable 0
    GPREN1          = 0x0050, // GPIO Pin Rising Edge Detect Enable 1
    GPFEN0          = 0x0058, // GPIO Pin Falling Edge Detect Enable 0
    GPFEN1          = 0x005c, // GPIO Pin Falling Edge Detect Enable 1
    GPHEN0          = 0x0064, // GPIO Pin High Detect Enable 0
    GPHEN1          = 0x0068, // GPIO Pin High Detect Enable 1
    GPLEN0          = 0x0070, // GPIO Pin Low Detect Enable 0
    GPLEN1          = 0x0074, // GPIO Pin Low Detect Enable 1
    GPAREN0         = 0x007c, // GPIO Pin Async. Rising Edge Detect 0
    GPAREN1         = 0x0080, // GPIO Pin Async. Rising Edge Detect 1
    GPAFEN0         = 0x0088, // GPIO Pin Async. Falling Edge Detect 0
    GPAFEN1         = 0x008c, // GPIO Pin Async. Falling Edge Detect 1
    GPPUD           = 0x0094, // GPIO Pin Pull-up/down Enable
    GPPUDCLK0       = 0x0098, // GPIO Pin Pull-up/down Enable Clock 0
    GPPUDCLK1       = 0x009c  // GPIO Pin Pull-up/down Enable Clock 1
};

enum {
    BCM_PASSWORD    = 0x5A000000
};


GPIOBase::pwm_vec_t GPIOBase::pwm_reg = {
    { 0,                0                   }, // 0
    { 0,                0                   },
    { 0,                0                   },
    { 0,                0                   },
    { 0,                0                   },
    { 0,                0                   }, // 5
    { 0,                0                   },
    { 0,                0                   },
    { 0,                0                   },
    { 0,                0                   },
    { 0,                0                   }, // 10
    { 0,                0                   },
    { FSEL::ALT0,       PWM_CHANNEL::CH0    },
    { FSEL::ALT0,       PWM_CHANNEL::CH1    },
    { 0,                0                   },
    { 0,                0                   }, // 15
    { 0,                0                   },
    { 0,                0                   },
    { FSEL::ALT5,       PWM_CHANNEL::CH0    },
    { FSEL::ALT5,       PWM_CHANNEL::CH1    },
    { 0,                0                   }, // 20
    { 0,                0                   },
    { 0,                0                   },
    { 0,                0                   },
    { 0,                0                   },
    { 0,                0                   }, // 25
    { 0,                0                   },
    { 0,                0                   },
    { 0,                0                   },
    { 0,                0                   },
    { 0,                0                   }, // 30
    { 0,                0                   }
};

unsigned int                GPIOBase::pwm_range_val[GPIOBase::PWM_CHANNELS] = { 0, 0 };
PWM_MODE                    GPIOBase::pwm_mode_val[GPIOBase::PWM_CHANNELS]  = { PWM_MODE::INVALID, PWM_MODE::INVALID };
unsigned int                GPIOBase::pwm_clock_val                         = 0; // one clock for all channels..
GPIOBase::soft_thread_map_t GPIOBase::soft_thread_map;
Mutex                       GPIOBase::m_soft_thread_mutex;

void GPIOBase::int_pwm_write(PWM_CHANNEL channel, unsigned int value)
{
    *(pwm_addr + ((channel == PWM_CHANNEL::CH0) ? PWM0_DATA : PWM1_DATA)) = value;
}

void GPIOBase::int_set_pwm_mode(PWM_CHANNEL channel, PWM_MODE mode)
{
    // check if that mode is already set

    if (pwm_mode_val[static_cast<int>(channel)] == mode) return;
    pwm_mode_val[static_cast<int>(channel)] = mode;

    // read the existing setting (as the register addresses both PWM channels)

    unsigned long control = *(pwm_addr + PWM_CONTROL);
    
    if (mode == PWM_MODE::MARKSPACE) {
        control |= (channel == PWM_CHANNEL::CH0) ? PWM_INT_MODE::PWM0_MS_MODE : PWM_INT_MODE::PWM1_MS_MODE;
    } else {
        control &= (channel == PWM_CHANNEL::CH0) ? ~PWM_INT_MODE::PWM0_MS_MODE : ~PWM_INT_MODE::PWM1_MS_MODE;
    }
    control |= (channel == PWM_CHANNEL::CH0) ? PWM0_ENABLE : PWM1_ENABLE;

    // write the new setting

    *(pwm_addr + PWM_CONTROL) = control;
}

void GPIOBase::int_set_pwm_range(PWM_CHANNEL channel, unsigned int range)
{
    // check if that range is already set

    if (pwm_range_val[static_cast<int>(channel)] == range) return;
    pwm_range_val[static_cast<int>(channel)] = range;

    // set the new range value

    *(pwm_addr + ((channel == PWM_CHANNEL::CH0) ? PWM0_RANGE : PWM1_RANGE)) = range;
}

void GPIOBase::int_set_pwm_clock(PWM_CHANNEL channel, unsigned int divisor)
{
    // neglect channel - there is only one clock for all (both) channels
    
    // check if this clock divider is already set

    if (divisor == pwm_clock_val) return;
    
    // store the new clock mode

    pwm_clock_val = divisor;

    unsigned long pwm_control = *(pwm_addr + PWM_CONTROL);

    // stop pwm when changing the clock divider

    *(pwm_addr + PWM_CONTROL) = 0;
    
    // stop pwm clock

    *(clk_addr + PWMCLK_CNTL) = BCM_PASSWORD | 0x01;
    std::this_thread::sleep_for(std::chrono::microseconds(150));
    
    // wait until clock is no more BUSY

    while ((*(clk_addr + PWMCLK_CNTL) & 0x80) != 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }

    // set the clock divider (clock runs at 19.2 Mhz)

    *(clk_addr + PWMCLK_DIV)  = BCM_PASSWORD | ((divisor & 0xfff) << 12) ;

    // start the pwm clock again

    *(clk_addr + PWMCLK_CNTL) = BCM_PASSWORD | 0x11;
    
    // and reset the pwm mode

    *(pwm_addr + PWM_CONTROL) = pwm_control;
}

void GPIOBase::int_set_mode(unsigned int gpio, FSEL mode)
{
    // first clear all bits for this gpio by declaring it an input
    // (and thus removing all ALT and output bits)

    *(gpio_addr + ((gpio) / 10)) &= ~(7 << (((gpio)%10)*3));
    
    if (mode != FSEL::INPUT) {

        // then set the mode

        *(gpio_addr + (((gpio) / 10))) |= (static_cast<unsigned long>(mode) << (((gpio)%10)*3));
    }
}

void GPIOBase::mode(unsigned int gpio, GPIO_MODE mode) const
{
    if (gpio >= 64) throw GPIOError("invalid gpio number");

    {
        // remove a soft thread if one is running at this gpio

        Lock lock(m_soft_thread_mutex);
        auto it = soft_thread_map.find(gpio);
        if (it != soft_thread_map.end()) soft_thread_map.erase(it);
    }

    if (gpio_only) {

        // fall back to soft pwm modes if we do not have access to the PWM control registers

        if (mode == GPIO_MODE::PWM_OUTPUT) mode = GPIO_MODE::SOFT_PWM_OUTPUT;
        else if (mode == GPIO_MODE::PWM_TONE_OUTPUT) mode = GPIO_MODE::SOFT_TONE_OUTPUT;
    }
    
    switch (mode) {

        case GPIO_MODE::GPIO_INPUT:
        {
            int_set_mode(gpio, FSEL::INPUT);
            break;
        }

        case GPIO_MODE::GPIO_OUTPUT:
        {
            int_set_mode(gpio, FSEL::OUTPUT);
            break;
        }
            
        case GPIO_MODE::PWM_OUTPUT:
        case GPIO_MODE::PWM_TONE_OUTPUT:
        {
            check_hard_pwm(gpio);
            int_set_mode(gpio, pwm_reg[gpio].fsel);
            std::this_thread::sleep_for(std::chrono::microseconds(150));
            int_set_pwm_mode(pwm_reg[gpio].channel, (mode == GPIO_MODE::PWM_TONE_OUTPUT) ? PWM_MODE::MARKSPACE : PWM_MODE::BALANCED);
            int_set_pwm_range(pwm_reg[gpio].channel, 1024);

            // only set the clock divider if it has not already been set - otherwise leave it untouched!

            if (pwm_clock_val == 0) int_set_pwm_clock(pwm_reg[gpio].channel, 32);
            break;
        }

        case GPIO_MODE::SOFT_PWM_OUTPUT:
        {
            int_set_mode(gpio, FSEL::OUTPUT);
            Lock lock(m_soft_thread_mutex);
            soft_thread_map.emplace(gpio, std::make_unique<SoftThread>(GPIO_MODE::SOFT_PWM_OUTPUT, gpio, 100, 0));
            break;
        }
        
        case GPIO_MODE::SOFT_TONE_OUTPUT:
        {
            int_set_mode(gpio, FSEL::OUTPUT);
            Lock lock(m_soft_thread_mutex);
            soft_thread_map.emplace(gpio, std::make_unique<SoftThread>(GPIO_MODE::SOFT_TONE_OUTPUT, gpio, 5000, 0));
            break;
        }
            
        default:
            throw GPIOError("mode not supported");

    }
}

void GPIOBase::pullupdown(unsigned int gpio, GPIO_PULL pud) const
{
    Lock lock(pullupdown_mutex);
    *(gpio_addr + GPPUD/4) = static_cast<unsigned long>(pud);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    *(gpio_addr + GPPUDCLK0/4 + gpio/32) = 1 << gpio % 32;
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    *(gpio_addr + GPPUD/4) = static_cast<unsigned long>(GPIO_PULL::OFF);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    *(gpio_addr + GPPUDCLK0/4 + gpio/32) = 0;
    std::this_thread::sleep_for(std::chrono::microseconds(10));
}

bool GPIOBase::has_hard_pwm(unsigned int gpio) const
{
    return !gpio_only && gpio < 32 && pwm_reg[gpio].fsel != FSEL::INPUT;
}

void GPIOBase::check_hard_pwm(unsigned int gpio) const
{
    if (!has_hard_pwm(gpio)) throw GPIOError("cannot set this gpio to hard PWM mode");
}

void GPIOBase::pwm_write(unsigned int gpio, unsigned int value) const
{
    check_hard_pwm(gpio);
    int_pwm_write(pwm_reg[gpio].channel, value);
}

void GPIOBase::pwm_tone_write(unsigned int gpio, unsigned int frequency) const
{
    if (frequency == 0) pwm_write(gpio, 0);
    else {
        unsigned int range = 19200000 / pwm_clock_val / frequency;
        pwm_range(gpio, range);
        pwm_write(gpio, frequency / 2);
    }
}

void GPIOBase::pwm_mode(unsigned int gpio, PWM_MODE mode) const
{
    check_hard_pwm(gpio);
    int_set_pwm_mode(pwm_reg[gpio].channel, mode);
}

void GPIOBase::pwm_range(unsigned int gpio, unsigned int range) const
{
    check_hard_pwm(gpio);
    int_set_pwm_range(pwm_reg[gpio].channel, range);
}

void GPIOBase::pwm_clock(unsigned int gpio, unsigned int divisor) const
{
    check_hard_pwm(gpio);
    int_set_pwm_clock(pwm_reg[gpio].channel, divisor);
}

void GPIOBase::soft_pwm_range(unsigned int gpio, unsigned int range) const
{
    Lock lock(m_soft_thread_mutex);
    auto it = soft_thread_map.find(gpio);
    if (it == soft_thread_map.end()) throw GPIOError("no soft pwm on this gpio");
    it->second->set_range(range);
}

void GPIOBase::soft_pwm_write(unsigned int gpio, unsigned int value) const
{
    Lock lock(m_soft_thread_mutex);
    auto it = soft_thread_map.find(gpio);
    if (it == soft_thread_map.end()) throw GPIOError("no soft pwm on this gpio");
    it->second->set_value(value);
}

void GPIOBase::soft_tone_write(unsigned int gpio, unsigned int frequency) const
{
    if (frequency > 10000) throw GPIOError("frequency max 10000 (Hz)");
    soft_pwm_write(gpio, frequency);
}

GPIOBase::GPIOBase()
{
    std::call_once(initlock, init);
}

void GPIOBase::simulation(bool yes)
{
    simulate = yes;
}


void GPIOBase::SoftThread::run_pwm()
{
    static const std::chrono::microseconds base_period = std::chrono::microseconds(100);
    
    for (;;) {
        unsigned int local_mark = m_mark;
        unsigned int local_space = m_space;

        // terminate?

        if (local_mark == 0 && local_space == 0) return;
        if (local_mark != 0) {
            int_set(m_gpio);
            std::this_thread::sleep_for(base_period * local_mark);
        }
        if (local_space != 0) {
            int_clear(m_gpio);
            std::this_thread::sleep_for(base_period * local_space);
        }
    }
}

void GPIOBase::SoftThread::run_tone()
{
    for (;;) {
        unsigned int local_mark = m_mark;
        // terminate?
        if (local_mark == 0 && m_space == 0) return;
        if (local_mark == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        else {
            int_set(m_gpio);
            std::this_thread::sleep_for(std::chrono::microseconds(500000 / local_mark));
            int_clear(m_gpio);
            std::this_thread::sleep_for(std::chrono::microseconds(500000 / local_mark));
        }
    }
}

void GPIOBase::SoftThread::set_range(unsigned int range)
{
    m_range = range;
    if (m_value > range) set_value(range);
}

void GPIOBase::SoftThread::set_value(unsigned int value)
{
    if (value > m_range) throw GPIOError("value > range");
    m_value = value;
    m_mark = value;
    m_space = m_range - m_mark;
}

GPIOBase::SoftThread::SoftThread(GPIO_MODE what, unsigned int gpio, unsigned int range, unsigned int value)
: m_range(range)
, m_value(value)
, m_gpio(gpio)
{
    if (m_range == 0) throw GPIOError("range == 0");
    if (m_value > m_range) throw GPIOError("value > range");

    // compute the mark and space values

    set_value(m_value);

    switch (what) {
        case GPIO_MODE::SOFT_PWM_OUTPUT:
            m_thread = std::make_unique<std::thread>(&GPIOBase::SoftThread::run_pwm, this);
            break;
        case GPIO_MODE::SOFT_TONE_OUTPUT:
            m_thread = std::make_unique<std::thread>(&GPIOBase::SoftThread::run_tone, this);
            break;
        default: throw GPIOError("invalid mode");
    }
}

GPIOBase::SoftThread::~SoftThread()
{
    if (!m_thread) return;

    // send a terminate message to the running thread

    terminate();

    // it will terminate in a wink

    m_thread->join();

    // do not delete the thread, it is a unique_ptr and cleans up itself
}

bool GPIOBase::force_full_mapping()
{
    // already initialized? then check in which mode

    if (peripherals_base || gpio_addr) return !gpio_only;

    want_full_mapping = true;

    // return true, even if the initialization might still fail if we are not root (with an exception)

    return true;
}

void GPIOBase::init()
{
    if (!peripherals_base) {

#ifndef __linux
        // this GPIO is only existing in Linux type operating systems. When compiling e.g. on a Mac,
        // force simulation mode for debugging.

        simulate = true;
#endif

        // check if we are in simulation mode (probably on another hardware, like a PC or a Mac)

        if (simulate) {

            peripherals_base = 0x20000000;
            peripherals_size = 0x01000000;

            peripherals_sim = std::make_unique<unsigned long[]>(peripherals_size / 4);
            peripherals = peripherals_sim.get();

        } else {

            // use /dev/gpiomem if available (does not need root, only group gpio, but we lose access on
            // PWM and all other buses than gpio.
            // however - as all higher level protocols are well handled by Linux modules, this should not
            // be an issue. Only problem is the pwm clock frequency, which cannot be set through a kernel
            // module. See /boot/overlay/README.)

            Tools::AutoFile fd;

            if (!want_full_mapping && fd.open_nothrow("/dev/gpiomem", O_RDWR | O_SYNC)) {

                gpio_addr = static_cast<volatile unsigned long*>(::mmap(nullptr, GPIO_MEM_BLOCK_SIZE,
                                                                        (PROT_READ | PROT_WRITE),
                                                                        MAP_SHARED, *fd, 0));
                if (gpio_addr == MAP_FAILED) throw GPIOError("mmap failed on /dev/gpiomem");
                
                gpio_only = true;

            } else {

                // Try to determine peripherals_base and peripherals_size

                // The peripherals base address is announced in /proc/device-tree/soc/ranges with
                // an offset of 4 and a size of 4, followed by the size.

                // If the file does not exist it shall be assumed 0x20000000 and the size with 0x01000000
                // (which was the address on the raspi 1 versions)

                if (!fd.open_nothrow("/proc/device-tree/soc/ranges", O_RDONLY)) {

                    // this is apparently a raspi 1 with an old kernel

                    peripherals_base = 0x20000000;
                    peripherals_size = 0x01000000;

                } else {

                    // else read base address at pos 4
                    fd.seek(4, SEEK_SET);
                    fd.read(peripherals_base);
                    fd.read(peripherals_size);

                    fd.close();

                    Tools::byteswap(peripherals_base);
                    Tools::byteswap(peripherals_size);

                }

                if (!peripherals_base) throw GPIOError("cannot read SOC peripherals base address");
                if (!peripherals_size) throw GPIOError("cannot read SOC peripherals size");

                fd.open("/dev/mem", O_RDWR | O_SYNC);

                peripherals = static_cast<volatile unsigned long*>(::mmap(nullptr, peripherals_size,
                                                                          (PROT_READ | PROT_WRITE),
                                                                          MAP_SHARED,
                                                                          *fd, peripherals_base));
                if (peripherals == MAP_FAILED) throw GPIOError("mmap failed on /dev/mem");

            }

        }

        if (gpio_only) {

            peripherals = nullptr;
            pwm_addr    = nullptr;
            clk_addr    = nullptr;
            pads_addr   = nullptr;
            spi0_addr   = nullptr;
            bsc0_addr   = nullptr;
            bsc1_addr   = nullptr;
            st_addr     = nullptr;

        } else {

            gpio_addr   = peripherals + GPIO_BASE  / sizeof(peripherals);
            pwm_addr    = peripherals + GPIO_PWM   / sizeof(peripherals);
            clk_addr    = peripherals + CLOCK_BASE / sizeof(peripherals);
            pads_addr   = peripherals + GPIO_PADS  / sizeof(peripherals);
            spi0_addr   = peripherals + SPI0_BASE  / sizeof(peripherals);
            bsc0_addr   = peripherals + BSC0_BASE  / sizeof(peripherals);
            bsc1_addr   = peripherals + BSC1_BASE  / sizeof(peripherals);
            st_addr     = peripherals + ST_BASE    / sizeof(peripherals);
            
        }
    }
}
