//
//  tools.cpp
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

#include <errno.h>
#include <unistd.h>
#include <memory>

#include "tools.hpp"

#if __cplusplus <= 201103L
// helper if the current compiler only supports C++11
#include "make_unique.hpp"
#endif



using namespace GPIO::Tools;

void AutoFile::open(const std::string& name, int mode)
{
    close();
    if ((m_fd = ::open(name.c_str(), mode)) < 0) throw ToolsError(std::string("cannot open file: ") + std::strerror(errno));
}

bool AutoFile::open_nothrow(const std::string& name, int mode) noexcept
{
    if (m_fd >= 0) {
        if (::close(m_fd) < 0) return false;
        m_fd = -1;
    }
    if ((m_fd = ::open(name.c_str(), mode)) < 0) return false;
    return true;
}

ssize_t AutoFile::read(void *buf, size_t count) const
{
    ssize_t rb = ::read(m_fd, buf, count);
    if (rb < 0) throw ToolsError(std::string("cannot read file: ") + std::strerror(errno));
    if (static_cast<size_t>(rb) != count) throw ToolsError("cannot read file, not all data read");
    return rb;
}

void AutoFile::write(const void *buf, size_t count) const
{
    ssize_t wb = ::write(m_fd, buf, count);
    if (wb < 0) throw ToolsError(std::string("cannot write file: ") + std::strerror(errno));
    if (static_cast<size_t>(wb) != count) throw ToolsError("cannot write file, not all data written");
}

off_t AutoFile::seek(off_t offset, int whence)
{
    off_t offs = ::lseek(m_fd, offset, whence);
    if (offs < 0) throw ToolsError(std::string("cannot seek file: ") + std::strerror(errno));
    return offs;
}

void AutoFile::close()
{
    if (m_fd < 0) return;
    if (::close(m_fd) < 0) throw ToolsError(std::string("cannot close file: ") + std::strerror(errno));
    m_fd = -1;
}

AutoFile::~AutoFile()
{
    if (m_fd >= 0) ::close(m_fd);
}

