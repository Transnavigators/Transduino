/**
 * Transduino
 *
 * The Arduino Code for the Transnavigators' Voice Controlled Wheelchair
 *
 * Input and output is over I2C
 *
 * Interfaces over SPI with a Dual LS7366R Quadrature Encoder Buffer
 * Interfaces over UART with a Sabertooth 2x60
 * 
 *
 * @author Transnavigators
 *
 */

#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <Encoder_Buffer.h>
#include <stdint.h>


#define SLAVE_ADDRESS 0x04
#define BAUD_RATE 115200

#define SABERTOOTH_ADDRESS 128
#define ENCODER1_SELECT_PIN 7
#define ENCODER2_SELECT_PIN 8
#define SW_SERIAL_PORT 2

// The number of bytes a command from the motor should be
// 1 byte for the motor command and 8 bytes for the motor speeds (2 floats, 4 bytes each)
#define MOVE_COMMAND_SIZE 9
#define MOVE_COMMAND 'm'
#define MOVE_TIMEOUT 500000 // Stop moving after 500000 us without receiving a command

// number of encoder pulses per meter (6 inch wheel diameter)
// 4096 pulses per revolution / (2*pi*(6 inches/2) * 0.0254 inches/meter)
#define PULSES_PER_METER 8555

// 2 ms delay incase we need to put a delay so that the motors' power does not oscillate to quickly
#define LOOP_DELAY 2

// Defining DEBUG switches to using software serial
// for communication with the Sabertooth Motor Controller
//
// Connect S1 of the Sabertooth to Pin SW_SERIAL_PORT in debug mode
// Connect S1 of the Sabertooth to Pin 1 when not in debug mode
#define DEBUG

// struct to send both encoder counts over SPI
typedef struct EncoderDataTag {
  int32_t encoder1Count;
  int32_t encoder2Count;
} EncoderData;

// union for retrieving floats over i2c
typedef union FloatUnionTag {
  byte bVal[4];
  float fVal;
} FloatUnion;

// the desired Speed of Motor1 and Motor2
volatile float Motor1Speed = 0;
volatile float Motor2Speed = 0;

// current power of Motor1 and Motor2
int8_t Motor1Power = 0;
int8_t Motor2Power = 0;

// the last time we received a message from the arduino
// used to timeout the arduino after MOVE_TIMEOUT microseconds
volatile uint32_t LastPiCommandTime = 0;

// this is the timestamp of the last iteration of the loop
uint32_t PreviousLoopTime = 0;

// holds current counts for the encoder
EncoderData data;

// initialize sabertooth
#ifndef DEBUG
Sabertooth ST(SABERTOOTH_ADDRESS);
#endif
#ifdef DEBUG
SoftwareSerial SWSerial(NOT_A_PIN, SW_SERIAL_PORT);
Sabertooth ST(SABERTOOTH_ADDRESS, SWSerial);
#endif

// initialize the encoders
Encoder_Buffer Encoder1(ENCODER1_SELECT_PIN);
Encoder_Buffer Encoder2(ENCODER2_SELECT_PIN);


/**
 * Setup routine
 */
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

/**
 *  Main loop
 */
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

/**
 * void receiveData(int byteCount)
 * 
 * Callback for receiving i2c data
 *
 * This function only accepts a message in the following format:
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


/**
 * void sendData()
 *
 * Callback for i2c data request
 *
 * Sends the current counts of the encoder
 *
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

