# AtariJoysticksPlus
Interface your old Atari joysticks, paddles, and driving controllers to a PC via Arduino.

Has been tested with an Arduino Leonardo. Requires the "Arduino Joystick" library, by Matthew Heironimus (MHeironimus), located at
https://github.com/MHeironimus/ArduinoJoystickLibrary

For connecting a basic Atari joystick, nothing much is needed: a DB9 male socket with wires connected to the Arduino pins. Atari driving paddles (rare, supplied with one Atari 2600 game) are automatically detected and supported via the Rx axis.

For connecting an Atari paddle pair as well as Wico and Epyx joysticks supporting a second (distinct) fire button, we also need three 1 Mega Ohm resistors.
