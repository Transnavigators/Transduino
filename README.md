# Transduino

[![Build Status](https://travis-ci.org/Transnavigators/Transduino.svg?branch=master)](https://travis-ci.org/Transnavigators/Transduino)

Transduino is Arduino code written for the Transnavigators' Voice Controlled Wheelchair.  It supports communication with a Raspberry Pi over I2C, with the Raspberry Pi configured as the master and the Arduino as the slave.  Over the interface, the Arduino accepts requests for setting the desired speeds of each motor and requests for getting the current counts of each encoder.  The Arduino interfaces with a [Sabertooth 2x60](https://www.dimensionengineering.com/products/sabertooth2x60) motor controller and a [Dual LS7366R Quadrature Encoder Buffer](https://www.superdroidrobots.com/shop/item.aspx/dual-ls7366r-quadrature-encoder-buffer/1523/).

## Arduino Setup and Configuration

## Required Hardware

* [Arduino Uno](https://store.arduino.cc/usa/arduino-uno-rev3)
* [Sabertooth 2x60](https://www.dimensionengineering.com/products/sabertooth2x60)
* [Dual LS7366R Quadrature Encoder Buffer](https://www.superdroidrobots.com/shop/item.aspx/dual-ls7366r-quadrature-encoder-buffer/1523/) (with [1024 P/R encoders](https://www.sparkfun.com/products/11102))

### Required Libraries

Copies of the required libraries are included in this repository in `/library/*`

* Note that the original sources of these files are from: [Sabertooth Library](https://www.dimensionengineering.com/software/SabertoothArduinoLibraries.zip) and [Encoder Buffer Library](https://github.com/SuperDroidRobots/Encoder-Buffer-Library.git)


## Raspberry Pi Interface (I2C)

The Arduino communicates with the Raspberry Pi over I2C.  The Raspberry Pi acts as the master and the Arduino acts as the slave.  Over the interface, the Arduino receives the speed each wheel should move at and also sends the current counts of the encoders.  The Arduino has been configured with an address of 0x04.

**Arduino's I2C address:** 0x04 (`SLAVE_ADDRESS`)

### Communication Data Format

1. Sending a speed (m/s) to the motors:

> * Write an 'm' (move) and two 32 bit float speeds (one for each motor) to the Arduino
>
> | Register (1 byte) | Motor 1 Speed (4 byte float)   | Motor 2 Speed (4 byte float)   |
> |:-----------------:|:------------------------------:|:------------------------------:|
> | 'm'               | Desired speed of Motor 1 (m/s) | Desired speed of Motor 2 (m/s) |

2. Getting the current encoder counts

> * Request data from the Arduino, optionally sending an 'e' (encoder)
>
> | Register (1 byte) |
> |:-----------------:|
> | 'e'               |
>
> * The Arduino will reply with
>
> | Encoder 1 Count (signed 32 bit integer)       | Encoder 2 Count (signed 32 bit integer)       |
> |:---------------------------------------------:|:---------------------------------------------:|
> | Cumulative number of pulses seen by Encoder 1 | Cumulative number of pulses seen by Encoder 2 |

## Motor Controller Interface (UART)

The Arduino communicates with the Sabertooth Motor Controller using its UART interface.  During normal operation, data is send from the Arduino's pin 1 (TX) to the Sabertooth's pin S1.  In debug mode, a Software Serial interface is used to send data to the motor controller in order to free the UART interface for printing debug messages over USB.  In this instance the Arduino sends the data using pin 2 (or whatever `SW_SERIAL_PORT` is set to).

The Sabertooth is physically configured with an address of 128 as is reflected in `SABERTOOTH_ADDRESS`.  Its baud rate (`BAUD_RATE`) has been set to 115200.

* **Sabertooth's address:**  128 (`SABERTOOTH_ADDRESS`)
* **Baud rate for UART interface:** 115200 (`BAUD_RATE`)
* **Sabertooth UART connection pin:** 1 (normal), 2 (debug `SW_SERIAL_PORT`)

For more information, consult the [Sabertooth 2x60 User Guide](https://www.dimensionengineering.com/datasheets/Sabertooth2x60.pdf)

## Encoder Interface (SPI)

The Arduino gets the counts of each encoder by interfacing with a Quadrature Encoder Buffer chip over SPI.  This chip is used because the [1024 P/R encoders](https://www.sparkfun.com/products/11102) send out pulses faster than the Arduino can count.  The Encoder Buffer chip counts the number of pulses for each encoder.
 The select pin for Encoder 1 is 7 (`ENCODER1_SELECT_PIN`) and the select pin for encoder 2 is pin 8 (`ENCODER2_SELECT_PIN`).

* **Encoder 1's select pin:** 7 (`ENCODER1_SELECT_PIN`)
* **Encoder 2's select pin:** 8 (`ENCODER2_SELECT_PIN`)


## Speed Control

The powers of motor is calculated using feedback from the encoders.  The current speed of the wheels is calculated every iteration of the main loop, and the powers of the each motor is adjusted accordingly by one level in the corresponding direction.  By using constant power changes, any glitches will not have much of an impact on the current speed of the the chair.

### Limitations:

* Encoder counts can wrap after ~87 minutes
* micros will wrap ~72 minutes
* Slow float operations