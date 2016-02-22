//
//  i2c.hpp
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

#ifndef i2c_hpp_IASZJGFISOLJDFPOSIDUHGZAUHIJDALSFASVYESF
#define i2c_hpp_IASZJGFISOLJDFPOSIDUHGZAUHIJDALSFASVYESF

#include <string>

namespace GPIO {

class I2C {
public:
    I2C(const std::string& interface, unsigned int device_id)
    {
        open(interface, device_id);
    }
    I2C() {}
    ~I2C()
    {
        close();
    }

    void open(const std::string& interface, unsigned int device_id);
    void close();

    int read() const;
    int regread8(int reg) const;
    int regread16(int reg) const;

    void write(int data) const;
    void regwrite8(int reg, int value) const;
    void regwrite16(int reg, int value) const;

private:
    int m_fd = -1;
};

}

#endif /* i2c_hpp */
