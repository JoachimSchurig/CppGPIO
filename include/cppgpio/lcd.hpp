//
//  lcd.hpp
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

#ifndef lcd_hpp_IAFHJADGFJHDSGFUZJGVUNKDLEUKENC
#define lcd_hpp_IAFHJADGFJHDSGFUZJGVUNKDLEUKENC

#include <string>
#include <vector>
#ifndef __GNUC__
#include <codecvt>
#endif
#include <cppgpio/gpio.hpp>
#include <cppgpio/i2c.hpp>


namespace GPIO {

    /// The class HitachiLCDBase  handles the actual communication with the LCD module,
    /// it does not store states, positions, etc. It is best used via its child class
    /// HitachiLCD, but you can also inherit to an own child class to write other functionality.
    /// See explanations at class HitachiLCD for the (protected) member functions
    
    class HitachiLCDBase : public ObjectBase {
    public:

        /// Constructs a HitachiLCDBase object, using 6 GPIO ports to connect to the
        /// display (4 data lines, and one RS and one EN / STROBE line).
        /// rows and cols specify the dimensions of the display, rows is limited from 1..4,
        /// cols are limited from 1..40 (and should of course match the physical dimensions
        /// of your display)

        HitachiLCDBase(unsigned int rs, unsigned int en,
                       unsigned int d4, unsigned int d5, unsigned int d6, unsigned int d7);

        /// Constructs a HitachiLCDBase object with the display connected via an I2C bus
        /// As the i2c expanders are sometimes connected with the lower nibble instead
        /// of the upper nibble to the display you can select this with upper_expander = false.
        /// Also, some of the displays with backlight toggle have the on/off logic inverted,
        /// which can be selected with inverted_backight_toggle = true
        /// rows and cols specify the dimensions of the display, rows is limited from 1..4,
        /// cols are limited from 1..40 (and should of course match the physical dimensions
        /// of your display)

        HitachiLCDBase(const std::string& i2c_interface, unsigned int i2c_device_id,
                       bool upper_expander = true, bool inverted_backlight_toggle = false);
        virtual ~HitachiLCDBase();

        /// switch backlight on or off. Not supported with all displays, and only via I2C

        void backlight(bool on);

    protected:
        GPIOBase gpio;
        I2C i2c;
        unsigned int m_rs;
        unsigned int m_en;
        unsigned int m_d4;
        unsigned int m_d5;
        unsigned int m_d6;
        unsigned int m_d7;
        unsigned int m_backlight;
        bool m_i2c = false;
        bool m_backlit = true;

        void int_pos(unsigned int row, unsigned int col);
        void int_write(unsigned char ch) { int_data(ch); }
        void int_chardef(unsigned int index, const unsigned char data[8]);
        void int_clear();
        void int_home();
        void int_display(bool display_on, bool cursor_on, bool cursor_blink);

    private:
        static unsigned char row_offset[4];
        bool m_upper_expander = true;
        bool m_inverted_backlight_toggle = false;

        void int_out_nibble(bool data, unsigned char out) const;
        void int_data(unsigned char out) const;
        void int_command_nibble(unsigned char out) const;
        void int_command(unsigned char out) const;
        void init_display();

    };
    

    
    /// The class HitachiLCD provides a high-level interface to the display driver class.
    /// It maintains a display buffer for being able to scroll lines up and down.
    ///
    /// The input charset is Unicode, either in UTF8 or in UTF32. Two types of
    /// character generators exist in the HD44780U chips: One with ASCII, some Western
    /// European, and some Japanese characters, and one with a mix of Western and Cyrillic
    /// characters. So we have to translate into this internal charset, and we have
    /// to work with the user character feature to add missing characters depending on
    /// the content of the current text buffer.
    /// The latter functionality is not yet implemented, instead, for latin based languages,
    /// a character replacement algorithm for accented characters with their unaccented
    /// sibling is performed. Only for German (and English of course) all characters are
    /// natively available from the built-in character generator.
    
    class HitachiLCD : public HitachiLCDBase {
    public:

        /// Constructs a HitachiLCD object, using 6 GPIO ports to connect to the
        /// display (4 data lines, and one RS and one EN / STROBE line).
        /// rows and cols specify the dimensions of the display, rows is limited from 1..4,
        /// cols are limited from 1..40 (and should of course match the physical dimensions
        /// of your display)

        HitachiLCD(unsigned int rows, unsigned int cols,
                   unsigned int rs, unsigned int en,
                   unsigned int d4, unsigned int d5,
                   unsigned int d6, unsigned int d7);

        /// Constructs a HitachiLCD object with the display connected via an I2C bus
        /// As the i2c expanders are sometimes connected with the lower nibble instead
        /// of the upper nibble to the display you can select this with upper_expander = false.
        /// Also, some of the displays with backlight toggle have the on/off logic inverted,
        /// which can be selected with inverted_backight_toggle = true
        /// rows and cols specify the dimensions of the display, rows is limited from 1..4,
        /// cols are limited from 1..40 (and should of course match the physical dimensions
        /// of your display)

        HitachiLCD(unsigned int rows, unsigned int cols,
                   const std::string& i2c_interface, unsigned int i2c_device_id,
                   bool upper_expander = true, bool inverted_backlight_toggle = false);

        /// destructor, resets all GPIO pins (if used) to input mode
        
        virtual ~HitachiLCD() {}

        /// returns number of columns of the display (1 based)

        unsigned int cols() const { return m_cols; }

        /// returns number of rows of the display (1 based)

        unsigned int rows() const { return m_rows; }

        /// wrap around at end of line, and scroll
        /// default off

        void wrap(bool on = true)
        {
            m_wrap = on;
        }

        /// expand with spaces until end of line
        /// default off

        void fill(bool on = true)
        {
            m_fill = on;
        }

        /// scroll display one line up or down

        void scroll(bool up = true);

        /// fill current line with spaces until end of line now, but do not change x y coordinates

        void fill_line();

        /// write count spaces at the current position (wrap if requested), but do not change x y coordinates

        void fill_space(unsigned int count);

        /// clear the display (and the internal buffer)

        void clear();

        /// move cursor to 0,0 (left upper corner) of the display without deleting any content

        void home();

        /// configure display properties

        void display(bool display_on, bool cursor_on, bool cursor_blink)
        {
            int_display(display_on, cursor_on, cursor_blink);
        }

        /// goto position row, col (0 based). 0, 0 is the left upper corner of the display

        void pos(unsigned int row, unsigned int col);

        /// write one single wide character (UTF32 Unicode) to the display

        void write(wchar_t ch);

        /// write a string (UTF8 Unicode) to the display. Returns count of real characters written.

        std::size_t write(const std::string& s);

        /// write a wide string (UTF32 Unicode) to the display. Returns count of real characters written.

        std::size_t write(const std::wstring& ws);

        /// write a string (UTF8 Unicode) at pos row, col to the display, fill with spaces to the right
        /// until width width of the string
        
        void write(unsigned int row, unsigned int col, const std::string& s, unsigned int width = 0);

        /// write a wide string (UTF32 Unicode) at pos row, col to the display, fill with spaces to the
        /// right until width width of the string

        void write(unsigned int row, unsigned int col, const std::wstring& ws, unsigned int width = 0);

        /// write one user defined character at index 0..7 to the character generator ram. Can be addressed
        /// in output strings with character codes 0..7 or 8..15. data gets the bitmask for the string, in 5x7

        void chardef(unsigned int index, const unsigned char data[8]);

    private:
        unsigned int m_rows;
        unsigned int m_cols;
        unsigned int m_x;
        unsigned int m_y;
        bool m_wrap;
        bool m_fill;

        // a recursive mutex to protext our high level functions from
        // concurrent access

        RecursiveMutex recursivemutex;

        // declare the type for the display buffer - as the display
        // is 8 bit, we use an 8 bit charset, too

        typedef std::vector<std::string> stringvec_t;

        // and create the buffer

        stringvec_t m_lines;

#ifndef __GNUC__
        // a converter to decode utf8 sequences

        std::wstring_convert<std::codecvt_utf8<wchar_t>> utf8convert;
#endif
        /// print display buffer to display

        void print_buffer();

        /// init variables

        void init(unsigned int rows, unsigned int cols);

        /// substitute umlauts by the character table position in the hitachi ROM

        bool substitute(wchar_t& ch);

        /// generate custom characters for non-substituable chars (if possible, else replace with nearest approximation)

        void generate(wchar_t& ch);
    };
    
}

#endif /* lcd_hpp */
