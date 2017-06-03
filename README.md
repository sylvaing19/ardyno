# ardyno

Teensy 3.x library for dynamixel servos

This library allows you to control the Robotis servo motors that use a custom half-duplex serial protocol. 
Communication speed up to 1 MBd is supported with hardware serial.
The most useful functions (speed, position, wheel/joint mode, ...) are provided via a very simple high level interface (see test_motor example), but other operations can be done using the generic read/write functions (see test_led example).

## Usage

To control TTL motors, it is recommended to use a logic level converter to convert the 3v3 of your teensy to the 5v used by Dynamixel servos. If your teensy board is 5V tolerant (teensy3.2 & 3.5) you can connect it directly the the servos but it will be less reliable (you may have a lot of communication errors).
Example of level converter : [SparkFun logic level converter](https://www.sparkfun.com/products/12009)

To control TTL/RS485 motors, with an additionnal hardware buffer:
See [Robotis documentation](http://support.robotis.com/en/)

## Troubleshooting

- If the servo is configured to answer immediatly to commands, the response packet may come back too fast and be missed by the teensy, and communication may become unstable or even impossible. To fix this, you have to write the value of the "Return Delay Time" register back to its orginal value (register 0x05, value 250).
