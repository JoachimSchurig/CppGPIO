//
//  lcd.cpp
//
//  Wrapper classes for Hitachi HD44780U LCD driver compatible chips
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

#include <string>
#include <chrono>

#include <cppgpio/lcd.hpp>
#include <cppgpio/i2c.hpp>

#include "tools.hpp"


using namespace GPIO;


enum COMMANDS {
    LCD_CLEAR           = 0x01,
    LCD_HOME            = 0x02,
    LCD_ENTRY           = 0x04,
    LCD_CTRL            = 0x08,
    LCD_CDSHIFT         = 0x10,
    LCD_FUNC            = 0x20,
    LCD_CGRAM           = 0x40,
    LCD_DGRAM           = 0x80
};

// bits in the entry register

enum ENTRY_REG {
    LCD_ENTRY_SH        = 0x01,
    LCD_ENTRY_ID        = 0x02
};

// bits in the control register

enum CTRL_REG {
    LCD_BLINK_CTRL      = 0x01,
    LCD_CURSOR_CTRL     = 0x02,
    LCD_DISPLAY_CTRL    = 0x04
};

// bits in the function register

enum FUNC_REG {
    LCD_FUNC_F          = 0x04,
    LCD_FUNC_N          = 0x08,
    LCD_FUNC_DL         = 0x10
};

// shift RTL or LTR?

enum LCD_DIRECTION {
    LCD_CDSHIFT_RL      = 0x04
};

enum I2C_EXPANDER {
    RS = 0x1,
    RW = 0x2,
    EN = 0x4,
    BL = 0x8
};

unsigned char HitachiLCDBase::row_offset[4] = {
    0x00,
    0x40,
    0x14,
    0x54
};

void HitachiLCDBase::int_out_nibble(bool data, unsigned char out) const
{
    if (!m_i2c) {

        // display directly attached to GPIO

        gpio.write(m_rs, data);
        gpio.write(m_d4, out & 1);
        out >>= 1;
        gpio.write(m_d5, out & 1);
        out >>= 1;
        gpio.write(m_d6, out & 1);
        out >>= 1;
        gpio.write(m_d7, out & 1);

        // send enable (strobe) to display

        gpio.set(m_en);
        std::this_thread::sleep_for(std::chrono::microseconds(55));

        // clear enable (the display takes the data on the falling edge)

        gpio.clear(m_en);
        std::this_thread::sleep_for(std::chrono::microseconds(55));

    } else {

        // display attached on I2C bus

        unsigned char ctrl = out;
        if (m_upper_expander) {
            ctrl <<= 4;
        }
        if (data) ctrl |= m_rs;
        if (m_inverted_backlight_toggle) {
            ctrl |= ((m_backlit) ? 0 : m_backlight);
        } else {
            ctrl |= ((m_backlit) ? m_backlight : 0);
        }

        // send enable (strobe)

        i2c.write(m_en | ctrl);

        // and clear strobe (the display takes the data on the falling edge)

        i2c.write(ctrl);

    }
}

void HitachiLCDBase::int_data(unsigned char out) const
{
    int_out_nibble(true, out >> 4);
    int_out_nibble(true, out & 0x0F);
}

void HitachiLCDBase::int_command_nibble(unsigned char out) const
{
    int_out_nibble(false, out);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

void HitachiLCDBase::int_command(unsigned char out) const
{
    int_out_nibble(false, out >> 4);
    int_out_nibble(false, out & 0x0F);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

/*** medium level functions using above low level infrastructure ***/

void HitachiLCDBase::int_pos(unsigned int row, unsigned int col)
{
    // limit row to 0..3
    row &= 0b11;
    int_command(col + (LCD_DGRAM | row_offset[row]));
}

void HitachiLCDBase::int_chardef(unsigned int index, const unsigned char data[8])
{
    if (index > 7) throw GPIOError("index in int_chardef > 7");

    int_command(LCD_CGRAM | ((index & 7) << 3));

    for (int i = 0 ; i < 8 ; ++i) int_data(data[i]);
}

void HitachiLCDBase::int_clear()
{
    int_command(LCD_CLEAR);
    int_command(LCD_HOME);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
}

void HitachiLCDBase::int_home()
{
    int_command(LCD_HOME);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
}

void HitachiLCDBase::int_display(bool display_on, bool cursor_on, bool cursor_blink)
{
    unsigned char cmd = LCD_CTRL;
    if (display_on)     cmd |= LCD_DISPLAY_CTRL;
    if (cursor_on)      cmd |= LCD_CURSOR_CTRL;
    if (cursor_blink)   cmd |= LCD_BLINK_CTRL;
    int_command(cmd);
}

void HitachiLCDBase::backlight(bool on)
{
    if (on == m_backlit) return;
    m_backlit = on;

    // send a dummy command

    int_command(0);
}

void HitachiLCDBase::init_display()
{
    unsigned char func = LCD_FUNC | LCD_FUNC_DL;

    // set 4-bit 1-line mode 3 times

    int_command_nibble(func >> 4);
    int_command_nibble(func >> 4);
    int_command_nibble(func >> 4);

    // and switch to full 4-bit mode

    func = LCD_FUNC;
    int_command_nibble(func >> 4);

    int_command(LCD_ENTRY   | LCD_ENTRY_ID);
    int_command(LCD_CDSHIFT | LCD_CDSHIFT_RL);

    // and set display to init configuration

    int_display(true, false, false);
    int_clear();
}

HitachiLCDBase::HitachiLCDBase(unsigned int rs, unsigned int en,
                               unsigned int d4, unsigned int d5,
                               unsigned int d6, unsigned int d7)
: m_rs(rs)
, m_en(en)
, m_d4(d4)
, m_d5(d5)
, m_d6(d6)
, m_d7(d7)
, m_backlight(0)
{
    // prepare pins

    gpio.clear(m_rs);
    gpio.clear(m_en);
    gpio.clear(m_d4);
    gpio.clear(m_d5);
    gpio.clear(m_d6);
    gpio.clear(m_d7);

    // switch to output

    gpio.mode(m_rs, GPIO_MODE::GPIO_OUTPUT);
    gpio.mode(m_en, GPIO_MODE::GPIO_OUTPUT);
    gpio.mode(m_d4, GPIO_MODE::GPIO_OUTPUT);
    gpio.mode(m_d5, GPIO_MODE::GPIO_OUTPUT);
    gpio.mode(m_d6, GPIO_MODE::GPIO_OUTPUT);
    gpio.mode(m_d7, GPIO_MODE::GPIO_OUTPUT);

    init_display();
}

HitachiLCDBase::HitachiLCDBase(const std::string& i2c_interface, unsigned int i2c_device_id,
                               bool upper_expander, bool inverted_backlight_toggle)
: i2c(i2c_interface, i2c_device_id)
, m_rs((upper_expander) ? RS : RS << 4)
, m_en((upper_expander) ? EN : EN << 4)
, m_backlight((upper_expander) ? BL : BL << 4)
, m_i2c(true)
, m_upper_expander(upper_expander)
, m_inverted_backlight_toggle(inverted_backlight_toggle)
{
    init_display();
}

HitachiLCDBase::~HitachiLCDBase()
{
    if (!m_i2c) {
        gpio.mode(m_rs, GPIO_MODE::GPIO_INPUT);
        gpio.mode(m_en, GPIO_MODE::GPIO_INPUT);
        gpio.mode(m_d4, GPIO_MODE::GPIO_INPUT);
        gpio.mode(m_d5, GPIO_MODE::GPIO_INPUT);
        gpio.mode(m_d6, GPIO_MODE::GPIO_INPUT);
        gpio.mode(m_d7, GPIO_MODE::GPIO_INPUT);
    }
}

/*** end of low level configuration and access ***/






void HitachiLCD::print_buffer()
{
    RecursiveLock rlock(recursivemutex);
    for (unsigned int i = 0; i < m_rows; ++i) {
        int_pos(i, 0);
        for (auto ch : m_lines[i]) int_write(ch);
    }
    int_pos(m_y, m_x);
}

void HitachiLCD::clear()
{
    RecursiveLock rlock(recursivemutex);
    int_clear();
    m_x = m_y = 0;
    for (auto& it : m_lines) {
        it.assign(m_cols, ' ');
    }
}

void HitachiLCD::home()
{
    RecursiveLock rlock(recursivemutex);
    int_home();
    m_x = m_y = 0;
}

void HitachiLCD::scroll(bool up)
{
    RecursiveLock rlock(recursivemutex);
    if (up) {
        for (unsigned int i = 0; i < m_rows-1; ++i) {
            m_lines[i].swap(m_lines[i+1]);
        }
        m_lines[m_rows-1].assign(m_cols, ' ');
    } else {
        for (auto i = m_rows-1; i > 0; --i) {
            m_lines[i].swap(m_lines[i-1]);
        }
        m_lines[0].assign(m_cols, ' ');
    }
    print_buffer();
}

void HitachiLCD::pos(unsigned int row, unsigned int col)
{
    RecursiveLock rlock(recursivemutex);
    if (col >= m_cols) return;
    if (row >= m_rows) return;
    int_pos(row, col);
    m_x = col;
    m_y = row;
}

// substitutions for the characters from 0xa0 - 0xff

const wchar_t substmap[] = {
    0x20, // NBSP
    '!' , // inverted !
    0xec, // cent
    0xed, // pound
    0xed, //
    0xfa, // yen
    0x7c, // |
    '?' , // §
    0x22, // diacrites
    0xec, // Copy -> cent
    0x61, // a
    0x7f, // <<
    0xb0, // -
    0x2d, // soft hyphen
    0x72, // registered -> r
    0xb0, // -
    0xdf, // °
    '?' , // +-
    0x32, // 2
    0x33, // 3
    0x60, // '
    0xe4, // mu
    0xf1, // par
    0xa5, // .
    0x2c, // isolated cedille
    0x31, // 1
    0x30, // 0
    0x7e, // >>
    0x34, // 1/4 -> 4
    0x32, // 1/2 -> 2
    '?' , // 3/4 -> ?
    '?' , // inverted ?
    0x41, // As -> A
    0x41, // As -> A
    0x41, // As -> A
    0x41, // As -> A
    0xe1, // Ä
    0x41, // As -> A
    0x41, // As -> A
    0x43, // C cedille -> C
    0x45, // Es -> E
    0x45, // Es -> E
    0x45, // Es -> E
    0x45, // Es -> E
    0x49, // Is -> I
    0x49, // Is -> I
    0x49, // Is -> I
    0x49, // Is -> I
    0x44, // D  -> D
    0xee, // Ñ
    0x4f, // Os -> O
    0x4f, // Os -> O
    0x4f, // Os -> O
    0x4f, // Os -> O
    0xef, // Ö
    0xf8, // x
    0x4f, // Os -> O
    0x55, // Us -> U
    0x55, // Us -> U
    0x55, // Us -> U
    0xf5, // Ü
    0x59, // Y -> Y
    '?' , //
    0xe2, // ß
    0x61, // as -> a
    0x61, // as -> a
    0x61, // as -> a
    0x61, // as -> a
    0xe1, // ä
    0x61, // as -> a
    0x61, // as -> a
    0x63, // c cedille -> c
    0x65, // es -> e
    0x65, // es -> e
    0x65, // es -> e
    0x65, // es -> e
    0x69, // is -> i
    0x69, // is -> i
    0x69, // is -> i
    0x69, // is -> i
    0x64, // d -> d
    0xee, // ñ
    0x6f, // os -> o
    0x6f, // os -> o
    0x6f, // os -> o
    0x6f, // os -> o
    0xef, // ö
    0xfd, // divider
    0x6f, // os -> o
    0x75, // us -> u
    0x75, // us -> u
    0x75, // us -> u
    0xf5, // ü
    0x79, // y -> y
    '?' , //
    0x79  // y -> y
};

bool HitachiLCD::substitute(wchar_t& ch)
{
    if (ch < 0x80) {

        // return true if ASCII, but find substitutions for backslash and tilde

        if (ch == 0x5c) ch = 0xa4; // backslash (not perfect, but acceptable)
        else if (ch == 0x7e) ch = 0xf3; // tilde
        return true;
    }

    // we cannot substitute the 0x80-0x9f block, and we provide no substitutions > 0xff

    if (ch < 0xa0 || ch > 0xff) return false;

    // get a substitution from the map

    ch = substmap[ch - 0xa0];
    return true;
}

void HitachiLCD::generate(wchar_t& ch)
{
    // replace by underscore to indicate unsupported character by now

    ch = '_';
}

/// We expect characters in Unicode, but cut it to the first 256 codepoints
/// (which is iso-8859-1 (and neither iso-8859-15 nor Windows 1252))

void HitachiLCD::write(wchar_t ch)
{
    RecursiveLock rlock(recursivemutex);

    // check if we can substitute some accented chars by their corresponding
    // positions in the LCD ROM else try to generate a custom character, or
    // approximate to nearest

    if (!substitute(ch)) generate(ch);

    // limit to character ROM range of the display if not already done by
    // substitute() or generate()

    ch &= 0x00ff;

    // check if we have to wrap / scroll

    if (m_x >= m_cols) {
        if (!m_wrap) return;
        m_x = 0;
        if (m_y >= m_rows-1) scroll(true);
        else ++m_y;
    }
    int_write(ch);
    m_lines[m_y][m_x] = ch;
    ++m_x;
}

void HitachiLCD::fill_space(unsigned int count)
{
    RecursiveLock rlock(recursivemutex);
    unsigned int x = m_x;
    for (unsigned int ct = 0; ct < count; ++ct) write(' ');
    pos(m_y, x);
}

void HitachiLCD::fill_line()
{
    RecursiveLock rlock(recursivemutex);
    fill_space(m_cols - m_x);
}

std::size_t HitachiLCD::write(const std::wstring& ws)
{
    RecursiveLock rlock(recursivemutex);
    for (auto ch : ws) write(ch);
    return ws.size();
}

std::size_t HitachiLCD::write(const std::string& s)
{
    RecursiveLock rlock(recursivemutex);
    //#ifndef __GNUC__
#if !defined(__GNUC__)
    std::wstring ws = utf8convert.from_bytes(s);
#else
    // GCC does not yet support <codecvt> ...
    std::wstring ws;
    //  ws += 'a'; // this crashes g++, at least in gnu++1y mode!
    ws.clear(); // therefore keep the clear() here.. it avoids the crash in from_utf8()
    Tools::from_utf8(s, ws);
#endif
    return write(ws);
}

void HitachiLCD::write(unsigned int row, unsigned int col, const std::string& s, unsigned int width)
{
    RecursiveLock rlock(recursivemutex);
    pos(row, col);
    std::size_t len = write(s);
    if (m_fill) fill_line();
    else if (width > len) fill_space(width - static_cast<unsigned int>(len));
}

void HitachiLCD::write(unsigned int row, unsigned int col, const std::wstring& ws, unsigned int width)
{
    RecursiveLock rlock(recursivemutex);
    pos(row, col);
    std::size_t len = write(ws);
    if (m_fill) fill_line();
    else if (width > len) fill_space(width - static_cast<unsigned int>(len));
}

void HitachiLCD::chardef(unsigned int index, const unsigned char data[8])
{
    RecursiveLock rlock(recursivemutex);
    index &= 0x7;
    int_chardef(index, data);
}

void HitachiLCD::init(unsigned int rows, unsigned int cols)
{
    m_rows = rows;
    m_cols = cols;
    m_x    = 0;
    m_y    = 0;
    m_wrap = false;
    m_fill = false;
    if (!m_rows || !m_cols || m_rows > 4 || m_cols > 40) throw GPIOError("range exceeded: rows 1..4, cols 1..40");
    m_lines.resize(m_rows);
    clear();
}

HitachiLCD::HitachiLCD(unsigned int rows, unsigned int cols,
                       unsigned int rs, unsigned int strobe,
                       unsigned int d4, unsigned int d5, unsigned int d6, unsigned int d7)
: HitachiLCDBase(rs, strobe, d4, d5, d6, d7)
{
    init(rows, cols);
}

HitachiLCD::HitachiLCD(unsigned int rows, unsigned int cols,
                       const std::string& i2c_interface, unsigned int i2c_device_id,
                       bool upper_expander, bool inverted_backlight_toggle)
: HitachiLCDBase(i2c_interface, i2c_device_id, upper_expander, inverted_backlight_toggle)
{
    init(rows, cols);
}

