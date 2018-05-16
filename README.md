# Mini 500V Driver for Capacitive Loads

This driver is based on a bidirectional flyback converter topology that is capable of driving 550V arbitrary waveforms to capacitive devices. It is controlled by Teensy 3.5/3.6 microcontrollers.


## Required Software
Arduino IDE: https://www.arduino.cc/en/Main/Software

Teensyduino Installation: https://www.pjrc.com/teensy/teensyduino.html


## Specifications
Input Voltage Range: 3.5V-4.5V (with reverse voltage protection)

Output Voltage Range: 0V-550V

Output Capacitance: 1nF-50nF (a broader range can work, but only these values were tested)

Note: Ensure trace between VUSB and VIN on Teensy is cut in order to allow the USB to be simultaneously plugged in with the input power. Failing to do so may destroy the Teensy. More details here: https://www.pjrc.com/teensy/external_power.html.


## Driver Code
Refer only to the code contained in the `master` branch.

`power_converter.ino` contains the main Arduino `setup()` and `loop()` functions.

`converter_lib.cpp` contains all driver initialization and low level control code.

`converter_lib.h` contains all constants and function prototypes.

