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

#include <iostream>
#include <cppgpio.hpp>


using namespace GPIO;


/// In this example we output text to an LCD display (and scroll it around)

void lcdtest()
{
// either configure the lcd via separate gpio pins
//    HitachiLCD lcd( 2, 16,
//                   26, 19,
//                   21, 20, 16, 12);
// or via the i2c bus
    HitachiLCD lcd(4, 20, "/dev/i2c-1", 0x27);

    for (unsigned int ct = 0; ct < lcd.rows(); ++ct) {
        lcd.write(ct, 0, std::string("Hello World ") + std::to_string(ct+1) + "!");
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    for (unsigned int ct = 0; ct < lcd.rows(); ++ct) {
        lcd.scroll(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    lcd.wrap();
    lcd.home();
    for (int i = 0; i < 30; ++i) {
        lcd.write("And a test. ");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    lcd.home();
    for (int i = 0; i < 30; ++i) {
        lcd.write("And a test. ");
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    
    {
        lcd.clear();
        std::string s = "1234567890abcdef";
        for (int x = 0; x < 50; ++x) {
            lcd.write(0, 0, s);
            std::string::value_type ch = s[0];
            s.erase(0, 1);
            s += ch;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
    
    {
        lcd.clear();
        std::string s1 = "The first line. ";
        std::string s2 = "And another one.";
        for (int x = 0; x < 50; ++x) {
            if ((x & 1) == 0) {
                lcd.write(0, 0, s1);
                lcd.write(1, 0, s2);
            } else {
                lcd.write(0, 0, s2);
                lcd.write(1, 0, s1);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
    
    lcd.clear();
    lcd.write(0, 0, "Goodbye!");
    
}


/// In the following example we actively poll a rotary dial with
/// integrated push button every 100ms. As the rotary dial is updating its
/// value in its own thread internally, we do not risk that it misses a turn.
/// However, we could miss a push. Therefore better use the event driven
/// approach from the Rotary1 example below.

void buttontest()
{
    // either configure the lcd via separate gpio pins
//    HitachiLCD lcd( 2, 16,
//                   26, 19,
//                   21, 20, 16, 12);
    // or via the i2c bus
    HitachiLCD lcd(4, 20, "/dev/i2c-1", 0x27);

    // create a RotaryDial object

    RotaryDial dial(6, 12, GPIO_PULL::UP);

    // limit the internal counter to a range from 0 to 100

    dial.set_range(0, 100);

    // and start the event detection of the dial

    dial.start();

    // create a PushButton object

    PushButton push(5, GPIO_PULL::UP);

    // and start the event detection of the push button

    push.start();
    
    lcd.fill();
    lcd.write(1, 0, "Hello!");
    
    long value = 0;
    bool pushed = false;
    for(;;) {
        long new_value = dial.get_value();
        if (value != new_value) {
            value = new_value;
            std::string out = "Value: ";
            out += std::to_string(value);
            lcd.write(0, 0, out);
        }
        bool new_push = push.is_on();
        if (new_push != pushed) {
            pushed = new_push;
            std::string out = "Button: ";
            out += pushed ? "pushed" : "released";
            lcd.write(1, 0, out);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
}


/// In the following example we use the event driven approach with a
/// rotary dial with integrated push button. We connect callables
/// (in this case member functions of the Rotary1 class) to the
/// rotary and button classes, and have them called whenever an
/// event occurs. Remark there is no polling, and no control loop.

class Rotary1 {
public:
    Rotary1()
// the constructor for a 2x16 display connected to 6 discrete GPIO pins:
//  : lcd(2, 16, 26, 19, 21, 20, 16, 12)
// the display could also be connected to the i2c bus, and have 4x20 characters display size
    : lcd(4, 20, "/dev/i2c-1", 0x27)
    , dial(6, 12, GPIO_PULL::UP)
    , push(5, GPIO_PULL::UP)
    {
        lcd.fill();
        lcd.write(0, 0, "Please dial me!");
        lcd.write(1, 0, "Will exit at #42");

        // register a lambda function at the dial to connect it to this class

        dial.f_dialed = [&](bool up, long value) { dialed(up, value); };

        // could also use std::bind():
        // dial.f_dialed = std::bind(&Rotary1::dialed, this, std::placeholders::_1, std::placeholders::_2);

        push.f_pushed = [&]() { pushed(); };

        push.f_released = [&](std::chrono::nanoseconds nano) { released(nano); };

        // after finishing the initialization of the event driven input objects
        // start the event threads of the input objects

        dial.start();
        push.start();
    }

private:
    HitachiLCD lcd;
    RotaryDial dial;
    PushButton push;
    
    void dialed(bool up, long value)
    {
        std::string out;
        if (value == 42) out = "Goodbye!";
        else {
            out = "Value: ";
            out += std::to_string(value);
        }
        lcd.write(0, 0, out);
        if (value == 42) {
            lcd.write(1, 0, "");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            lcd.backlight(false);
            exit(0);
        }
    }
    
    void pushed()
    {
        lcd.write(1, 0, "Button: pushed");
    }
    
    void released(std::chrono::nanoseconds nano)
    {
        lcd.write(1, 0, "Button: released");
    }
    
};


int main(int argc, const char * argv[])
{
    lcdtest();

//  buttontest();
    
    Rotary1 rotary;

    // do something (or just sleep)
    std::this_thread::sleep_for(std::chrono::hours(1));
    
    return 0;
}


