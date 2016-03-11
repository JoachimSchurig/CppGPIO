//
//  i2c.cpp
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

#include <cinttypes>
#include <cstring>
#include <cerrno>
#include <sys/ioctl.h>
#include <fcntl.h>
#ifdef HAVE_I2C_HEADERS
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#endif
#include <unistd.h>
#include <stdexcept>

#include <cppgpio/i2c.hpp>


using namespace GPIO;

class I2CError : public std::runtime_error
{
    using std::runtime_error::runtime_error;
};

#ifndef HAVE_I2C_HEADERS
enum {
    I2C_SLAVE                   = 0x0703,
    I2C_SMBUS                   = 0x0720,
    I2C_SMBUS_READ              = 1,
    I2C_SMBUS_WRITE             = 0,
    I2C_SMBUS_BLOCK_MAX         = 32,
    I2C_SMBUS_BYTE              = 1,
    I2C_SMBUS_BYTE_DATA         = 2,
    I2C_SMBUS_WORD_DATA         = 3,
};

union i2c_smbus_data {
    uint8_t byte;
    uint16_t word;
    uint8_t block[I2C_SMBUS_BLOCK_MAX + 2];
};

struct i2c_smbus_ioctl_data {
    uint8_t read_write;
    uint8_t command;
    uint32_t size;
    i2c_smbus_data *data;
};
#endif

static inline void i2c_smbus(int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
    i2c_smbus_ioctl_data args;
    args.read_write = rw;
    args.command = command;
    args.size = size;
    args.data = data;
    if (::ioctl(fd, I2C_SMBUS, &args)) throw I2CError(std::string("i2c/smbus error: ") + std::strerror(errno));
}

int I2C::read() const
{
    i2c_smbus_data data;
    i2c_smbus(m_fd, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);
    return data.byte;
}

int I2C::regread8(int reg) const
{
    i2c_smbus_data data;
    i2c_smbus(m_fd, I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA, &data);
    return data.byte;
}

int I2C::regread16(int reg) const
{
    i2c_smbus_data data;
    i2c_smbus(m_fd, I2C_SMBUS_READ, reg, I2C_SMBUS_WORD_DATA, &data);
    return data.word;
}

void I2C::write(int data) const
{
    i2c_smbus(m_fd, I2C_SMBUS_WRITE, data, I2C_SMBUS_BYTE, nullptr);
}

void I2C::regwrite8(int reg, int value) const
{
    i2c_smbus_data data;
    data.byte = value;
    i2c_smbus(m_fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE_DATA, &data);
}

void I2C::regwrite16(int reg, int value) const
{
    i2c_smbus_data data;
    data.word = value;
    i2c_smbus(m_fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_WORD_DATA, &data);
}

void I2C::open(const std::string& interface, unsigned int device_id)
{
    if ((m_fd = ::open(interface.c_str(), O_RDWR)) < 0) {}
    if (::ioctl(m_fd, I2C_SLAVE, device_id)) throw I2CError(interface + ": cannot open i2c/smbus: " + std::strerror(errno));
}

void I2C::close()
{
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

