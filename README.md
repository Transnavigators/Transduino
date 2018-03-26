[![Build Status](https://travis-ci.org/Transnavigators/Transduino.svg?branch=master)](https://travis-ci.org/Transnavigators/Transduino)
[![Documentation Status](https://readthedocs.org/projects/transduino/badge/?version=master)](http://transduino.readthedocs.io/en/master/?badge=master)

# Transduino

Transduino is Arduino code written for the Transnavigators' Voice Controlled Wheelchair.  It supports communication with a Raspberry Pi over I2C, with the Raspberry Pi configured as the master and the Arduino as the slave.  Over the interface, the Arduino accepts requests for setting the desired speeds of each motor and requests for getting the current counts of each encoder.  The Arduino interfaces with a [Sabertooth 2x60](https://www.dimensionengineering.com/products/sabertooth2x60) motor controller and a [Dual LS7366R Quadrature Encoder Buffer](https://www.superdroidrobots.com/shop/item.aspx/dual-ls7366r-quadrature-encoder-buffer/1523/).

## Arduino Setup and Configuration

### Required Hardware

* [Arduino Uno](https://store.arduino.cc/usa/arduino-uno-rev3)
* [Sabertooth 2x60](https://www.dimensionengineering.com/products/sabertooth2x60)
* [Dual LS7366R Quadrature Encoder Buffer](https://www.superdroidrobots.com/shop/item.aspx/dual-ls7366r-quadrature-encoder-buffer/1523/) (with [1024 P/R encoders](https://www.sparkfun.com/products/11102))

### Required Libraries

Copies of the required libraries are included in this repository in `/library/*`

* **Note:** The libraries are originally from [Sabertooth Library](https://www.dimensionengineering.com/software/SabertoothArduinoLibraries.zip) and [Encoder Buffer Library](https://github.com/SuperDroidRobots/Encoder-Buffer-Library.git)

### Raspberry Pi Setup

* **Important:** Due to the Raspberry Pi's clock streching bug, set the Pi's I2C baud rate to 25 kHz in order to prevent the Arduino from clock stretching.

## Debug Mode

When the debug macro is `#define DEBUG` is defined, the Arduino is placed in DEBUG mode.  In this mode, the user can view debug messages from the Arduino using the Serial Monitor.  Pin 2 (`SW_SERIAL_PORT`) is configured to use Software Serial to send data to the motor controller in order to free the Arduino Uno's sole UART interface for communication with the computer.  As a result the following change must be made when switching between debug and production mode:

* **Debug Mode:** Pin 2 -> S1
* **Production Mode:** Pin 1 (TX) -> S1

## Serial Interfaces

The Arduino uses I2C, UART, and SPI to communicate with the different components.

### Raspberry Pi Interface (I2C)

The Arduino communicates with the Raspberry Pi over I2C.  The Raspberry Pi acts as the master and the Arduino acts as the slave.  Over the interface, the Arduino receives the desired speed of each wheel and sends the current counts of each encoder.  The Arduino is configured with an address of 0x04.

* **Arduino's I2C address:** 0x04 (`SLAVE_ADDRESS`)

#### Data Format

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

### Motor Controller Interface (UART)

The Arduino communicates with the Sabertooth Motor Controller using a UART interface.  During normal operation, data is send from the Arduino's pin 1 (TX) to the Sabertooth's pin S1.  In debug mode, a Software Serial interface is used to send data to the motor controller in order to free the UART interface for printing debug messages over USB.  In this instance the Arduino transmits data using pin 2 (or whatever `SW_SERIAL_PORT` is set to).

The Sabertooth is physically configured with an address of 128 as is reflected in `SABERTOOTH_ADDRESS`.  Its baud rate (`BAUD_RATE`) has been set to 115200.

* **Sabertooth's address:**  128 (`SABERTOOTH_ADDRESS`)
* **Baud rate for UART interface:** 115200 (`BAUD_RATE`)
* **UART Tx pin:** 1 (normal), 2 (debug `SW_SERIAL_PORT`)

For more information, consult the [Sabertooth 2x60 User Guide](https://www.dimensionengineering.com/datasheets/Sabertooth2x60.pdf).

### Encoder Interface (SPI)

The Arduino communicates with the Quadrature Encoder Buffer over SPI.  The buffer chip is used to count pulses for each encoder because the [1024 P/R encoders](https://www.sparkfun.com/products/11102) send out pulses faster than the Arduino can count.

The select pin for Encoder 1 is 7 (`ENCODER1_SELECT_PIN`) and the select pin for encoder 2 is pin 8 (`ENCODER2_SELECT_PIN`).

* **Encoder 1's select pin:** 7 (`ENCODER1_SELECT_PIN`)
* **Encoder 2's select pin:** 8 (`ENCODER2_SELECT_PIN`)


## Speed Control

The power sent to each motor is calculated using feedback from the encoders.  The current speeds of each wheel are calculated every iteration of the main loop, and the powers of the each motor is adjusted accordingly by one level in the corresponding direction.  By using constant power changes, any errors caused by bad data or other glitching will have only a momentary effect on the system and not cause large fluctuations in the current speed of the chair.

In addition, a timeout (`MOVE_TIMEOUT`) is applied on the motors to stop the platform from moving if the Arduino has not receive a command in a long time.  This timeout is to prevent the the chair from moving if the something happens to the Raspberry Pi or the I2C interface.

### Limitations

The following are known potential issues with the code.  Problems arising from these issues may cause minor glitching but should not cause any major affects on the overall system.

1. Encoder Buffer wrapping
> * **Issue:** Encoder counts can wrap after ~87 minutes.
> * **Resolution:** The Raspberry Pi should account for this.
2. Arduino clock wrapping
> * **Issue:** micros() will wrap ~72 minutes which may result in a near zero current speed.
> * **Resolution:** Since the motor's power can only change by one level every loop iteration, this will have negligable effects on the performance of the system.
3. Slow operations
> * **Issue:** Slow float operations.
> * **Resolution:** If the code is too slow, we can replace all floating point operations with fixed point ones.
4. Wheels shake when the desired speed is 0 m/s
> * **Issue:** The actual speed of the wheels may oscillate around the desired speed.
> * **Resolution:** Tighten the gears or anything mechanical in the drive train that could be oscillating.  Increase the loop delay (`LOOP_DELAY`) or increase the load on the platform to increase damping.
