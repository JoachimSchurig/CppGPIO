# CppGPIO
C++14 GPIO library for Raspberry Pi and other embedded systems

### What is CppGPIO
CppGPIO is a C++ library for the GPIOs of embedded systems like the Raspberry Pi written entirely in a
modern dialect of C++ as of C++14.

With the current source it only works on the Pi, but it has enough abstraction to be ported to other boards.

CppGPIO is a pure C++14 library. You cannot use it from C nor from non-C++14 capable compilers.

It implements a high speed low level access on the GPIO pins of the embedded CPU, much like the C
libraries like [wiringPi](http://wiringpi.com) or the [bcm2835](http://www.airspayce.com/mikem/bcm2835/) library.

In addition it implements a number of hardware abstractions based on the low level model, such as PushButtons,
RotaryDials, LCD displays, and other outputs.

The library works event based for input objects. This means that you do not have to poll for state changes
of e.g. a switch, but can simply register a function from your own class that will be called from the object
when the state changes. This sounds complicated but is actually quite simple with C++11/14 features like lambda
expressions. The demo.cpp file has samples for this.

The speed of the library is at the upper limit of the hardware capabilities of the Raspberry Pi. Only 12 
nanoseconds are needed to switch an output pin on or off. This results in a 44.67 Mhz square wave output
just by soft control.

### How to install
Just clone a revision from here, and get into the source directory and type
```
make -j4
sudo make install
make demo
```


