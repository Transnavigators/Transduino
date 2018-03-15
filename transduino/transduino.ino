/*!
   \file transduino.ino
   \brief The Arduino code for the Transnavigators' Voice Controlled Wheelchair

  Input and output is over <a href="https://www.arduino.cc/en/Reference/Wire">I2C</a>

  Interfaces over SPI with a <a href="https://www.superdroidrobots.com/shop/item.aspx/dual-ls7366r-quadrature-encoder-buffer/1523/">Dual LS7366R Quadrature Encoder Buffer</a>

  Interfaces over UART with a <a href="https://www.dimensionengineering.com/products/sabertooth2x60">Sabertooth 2x60</a>

  \mainpage Transduino

*/

#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <Encoder_Buffer.h>
#include <stdint.h>

//! The I2C address of the Arduino
#define SLAVE_ADDRESS 0x04

//! The serial baud rate for debug messages and Sabertooth communication
#define BAUD_RATE 115200

/*! \def SABERTOOTH_ADDRESS
    \brief The current address of the motor controller

    The address can be changed using a switch on the motor controller
*/
#define SABERTOOTH_ADDRESS 128

//! The left encoder select pin number
#define ENCODER1_SELECT_PIN 7

//! The right encoder select pin number
#define ENCODER2_SELECT_PIN 8

//! The digital pin to use software serial with for debug mode
#define SW_SERIAL_PORT 2

/*! \def MOVE_COMMAND_SIZE
    \brief The number of bytes a command from the motor should be

    1 byte for the motor command and 8 bytes for the motor speeds (2 floats, 4 bytes each)
*/
#define MOVE_COMMAND_SIZE 9

//! The command byte for a move command
#define MOVE_COMMAND 'm'


//! Stop moving after 500000 us without receiving a command
#define MOVE_TIMEOUT 500000

/*! \def PULSES_PER_METER
    \brief The number of encoder pulses per meter (6 inch wheel diameter)
    4096 pulses per revolution / (2*pi*(6 inches/2) * 0.0254 inches/meter)
*/
#define PULSES_PER_METER 8555

//! 2 ms delay to stabilize the system and make sure the motor's power doesn't oscillate
#define LOOP_DELAY 2

/*! \def DEBUG
    \brief Define DEBUG to turn on debug mode

    Defining DEBUG switches communication with the motor controller to software serial
    Connect S1 of the Sabertooth to Pin SW_SERIAL_PORT in debug mode
    Connect S1 of the Sabertooth to Pin 1 when not in debug mode
*/
#define DEBUG

/*! \struct EncoderDataTag
    \brief A struct to send both encoder counts over SPI
*/
typedef struct EncoderDataTag {
  int32_t encoder1Count;
  int32_t encoder2Count;
} EncoderData;

/*! \union FloatUnionTag
    \brief A union for retrieving floats over I2C
*/
typedef union FloatUnionTag {
  byte bVal[4];
  float fVal;
} FloatUnion;

//! The desired speed of the left motor
float Motor1Speed = 0;

//! The desired speed of the right motor
float Motor2Speed = 0;

//! The current power of the left motor
int8_t Motor1Power = 0;

//! The current power of the right motor
int8_t Motor2Power = 0;

/*! \var LastPiCommandTime
    \brief The last time we received a message from the Arduino
    used to timeout the arduino after MOVE_TIMEOUT microseconds
 */
uint32_t LastPiCommandTime = 0;

//! The timestamp of the last iteration of the loop
uint32_t PreviousLoopTime = 0;

/*! \var data
    \brief Holds current counts for the encoder
 */
EncoderData data;

// initialize sabertooth
#ifndef DEBUG
//! The Sabertooth motor controller object from the <a href="https://www.dimensionengineering.com/software/SabertoothArduinoLibrary/html/index.html">Sabertooth library</a>
Sabertooth ST(SABERTOOTH_ADDRESS);
#endif
#ifdef DEBUG
//! The serial port being used for the motor controller
SoftwareSerial SWSerial(NOT_A_PIN, SW_SERIAL_PORT);

//! The Sabertooth motor controller object from the <a href="https://www.dimensionengineering.com/software/SabertoothArduinoLibrary/html/index.html">Sabertooth library</a>
Sabertooth ST(SABERTOOTH_ADDRESS, SWSerial);
#endif

//! The left encoder object from the <a href="https://github.com/SuperDroidRobots/Encoder-Buffer-Library">Encoder_Buffer library</a>
Encoder_Buffer Encoder1(ENCODER1_SELECT_PIN);

//! The right encoder object from the <a href="https://github.com/SuperDroidRobots/Encoder-Buffer-Library">Encoder_Buffer library</a>
Encoder_Buffer Encoder2(ENCODER2_SELECT_PIN);


//! Setup routine
void setup() {

  // start Serial
#ifdef DEBUG
  SWSerial.begin(BAUD_RATE);
#endif
  Serial.begin(BAUD_RATE);

  // start SPI
  SPI.begin();

  // Initialize encoders
  Encoder1.initEncoder();
  Encoder2.initEncoder();

  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

#ifdef DEBUG
  Serial.println("Ready!");
#endif
}

//! Main loop
void loop() {
  // get the current clock time
  uint32_t currentLoopTime = micros();

  // timeout if we haven't received a command in TIMEOUT milliseconds
  if (currentLoopTime - LastPiCommandTime >= MOVE_TIMEOUT) {
    Motor1Speed = 0;
    Motor2Speed = 0;
  }

  // calculate the time interval since the last iteration of the loop
  uint32_t deltaTime = currentLoopTime - PreviousLoopTime;

  // reset the current loop time
  PreviousLoopTime = currentLoopTime;

  // read encoders and negate the 2nd encoder count <- important
  int32_t currentEncoder1Count = Encoder1.readEncoder();
  int32_t currentEncoder2Count = -Encoder2.readEncoder();
  
  // get speeds of the encoders
  // change in encoder count / ((pulses / meter)(change in time in ms)(1000000ms / s)
  float encoder1Speed = (float)(currentEncoder1Count - data.encoder1Count) / (PULSES_PER_METER * deltaTime / 1000000);
  float encoder2Speed = (float)(currentEncoder2Count - data.encoder2Count) / (PULSES_PER_METER * deltaTime / 1000000);

  // assign data for encoder count
  data.encoder1Count = currentEncoder1Count;
  data.encoder2Count = currentEncoder2Count;

  // get difference between the desired and actual motor speed
  // (will be positive if we need to speed up and negative if we need to slow down)
  float motor1SpeedDiff = Motor1Speed - encoder1Speed;
  float motor2SpeedDiff = Motor2Speed - encoder2Speed;

  // increase motor power accordingly and saturate at [-127,127] per Sabertooth library documentation
  if (motor1SpeedDiff > 0 && Motor1Power != 127) {
    Motor1Power++;
  }
  else if (motor1SpeedDiff < 0 && Motor1Power != -127) {
    Motor1Power--;
  }
  if (motor2SpeedDiff > 0 && Motor2Power != 127) {
    Motor2Power++;
  }
  else if (motor2SpeedDiff < 0 && Motor2Power != -127) {
    Motor2Power--;
  }


#ifdef DEBUG
// prints desired speed, current speed, current motor power, and current encoder counts and loop speed
  Serial.print("Desired: ");
  Serial.print(Motor1Speed);
  Serial.print(" | ");
  Serial.println(Motor2Speed);

  Serial.print("Current: ");
  Serial.print(encoder1Speed);
  Serial.print(" | ");
  Serial.println(encoder2Speed);
  
  Serial.print("Encoder Count: ");
  Serial.print(data.encoder1Count);
  Serial.print(" | ");
  Serial.println(data.encoder2Count);
  
  Serial.print("Sending: ");
  Serial.print(Motor1Power);
  Serial.print(" | ");
  Serial.println(Motor2Power);

  Serial.print("Microseconds diff in loop: ");
  Serial.println(deltaTime);
#endif

  // send powers to motors
  ST.motor(1, Motor1Power);
  ST.motor(2, Motor2Power);

  // delay(LOOP_DELAY);
}

//! Callback for receiving I2C data
/*!
 * \param [in] byteCount the number of bytes received
 * This function only accepts messages in the following format:
 *
 * | Register (1 byte) | Motor 1 Speed (4 byte float)   | Motor 2 Speed (4 byte float)   |
 * |-------------------|--------------------------------|--------------------------------|
 * |        'm'        | Desired speed of Motor 1 (m/s) | Desired speed of Motor 2 (m/s) |
 *
 */
void receiveData(int byteCount) {

  // buffer to hold the commands read from the motor
  int8_t readBuffer[MOVE_COMMAND_SIZE];
  uint8_t i = 0;

  // read all byte from the I2C buffer
  // if we get the number of bits that we expect for a move command, save the data into the buffer
  // if not then just empty the buffer
  while (Wire.available()) {
    // makes sure we aren't getting junk from the i2c line
    if (byteCount == MOVE_COMMAND_SIZE) {
      // save the data into the buffer
      readBuffer[i++] = Wire.read();
    }
    else {
      // clear our buffer
      Wire.read();
    }
  }

  // make sure that we read the correct number of bytes and that we have a move command
  // and update desired motor speeds
  if (i == MOVE_COMMAND_SIZE && readBuffer[0] == MOVE_COMMAND) {
    
    
    // use a union to parse the speed
    FloatUnion motor1Speed, motor2Speed;
    motor1Speed.bVal[0] = readBuffer[1];
    motor1Speed.bVal[1] = readBuffer[2];
    motor1Speed.bVal[2] = readBuffer[3];
    motor1Speed.bVal[3] = readBuffer[4];
    motor2Speed.bVal[0] = readBuffer[5];
    motor2Speed.bVal[1] = readBuffer[6];
    motor2Speed.bVal[2] = readBuffer[7];
    motor2Speed.bVal[3] = readBuffer[8];

    Motor1Speed = motor1Speed.fVal;
    Motor2Speed = motor2Speed.fVal;

    // update the time we last received a message
    LastPiCommandTime = micros();
  }

#ifdef DEBUG
  // print the number of bytes received and the current speeds of the motor if they were sent
  Serial.print("Received ");
  Serial.print(byteCount);
  Serial.println(" bytes");
  if (byteCount == MOVE_COMMAND) {
    Serial.print("Motor 1 Speed: ");
    Serial.println(Motor1Speed);
    Serial.print("Motor 2 Speed: ");
    Serial.println(Motor2Speed);
  }
#endif
}

//! Callback for i2c data request
/*!
 * Sends the current counts of the encoder
 * | Encoder 1 Count (signed 32 bit integer)       | Encoder 2 Count (signed 32 bit integer)       |
 * |-----------------------------------------------|-----------------------------------------------|
 * | Cumulative number of pulses seen by Encoder 1 | Cumulative number of pulses seen by Encoder 2 |
 * 
 */
void sendData() {
  Wire.write((byte*)(&data), sizeof(EncoderData));

#ifdef DEBUG
//  Serial.print("Encoder 1 Count: ");
//  Serial.println(data.encoder1Count);
//  Serial.print("Encoder 2 Count: ");
//  Serial.println(data.encoder2Count);
#endif
}

