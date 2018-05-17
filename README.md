# Mini 500V Driver for Capacitive Loads

This driver is based on a bidirectional flyback converter topology that is capable of driving 550V arbitrary waveforms to capacitive devices. It is compatible with Teensy 3.5/3.6 microcontrollers. With some modification to the code, Teensy 3.2/3.1 microcontrollers are also compatible with the board.


## Required Software
Arduino IDE: https://www.arduino.cc/en/Main/Software

Teensyduino Installation: https://www.pjrc.com/teensy/teensyduino.html


## Specifications
Input Voltage Range: 3.5V-4.5V (with reverse voltage protection)

Output Voltage Range: 0V-550V

Output Capacitance: 1nF-50nF (a broader range can work, but only these values were tested)


## Board Image
![](board_im.jpeg?raw=true)

## Driver Code
Refer only to the code contained in the `master` branch.

`power_converter.ino` contains the main Arduino `setup()` and `loop()` functions.

`converter_lib.cpp` contains all driver initialization and low level control code.

`converter_lib.h` contains all constants and function prototypes.

## Important Notes Before Operating
* Ensure trace between VUSB and VIN on Teensy is cut in order to allow the USB to be simultaneously plugged in with the input power: https://www.pjrc.com/teensy/external_power.html. Failure to do so can destroy the circuit.

* Ensure a capacitive load is attached before operating the device. Open load operation can destroy the circuit.
