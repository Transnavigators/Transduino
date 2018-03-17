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


#define MOTOR_COMMAND_SIZE 3
#define MOTOR_COMMAND 'm'
#define MOTOR_TIMEOUT 500

/**
 * Defining DEBUG switches to using software serial
 * for communication with the Sabertooth Motor Controller
 * 
 * Connect S1 of the Sabertooth to Pin SW_SERIAL_PORT in debug mode
 * Connect S1 of the Sabertooth to Pin 1 when not in debug mode
 */
#define DEBUG

// struct to send both encoder counts over SPI
typedef struct EncoderDataTag {
    signed long encoder1Count;
    signed long encoder2Count;
} EncoderData;

// Power of Motor1 and Motor2
int8_t Motor1Power = 0;
int8_t Motor2Power = 0;

// The last time we received a messge from the arduino
// used to timeout the arduino after MOTOR_TIMEOUT milliseconds
long LastPiCommandTime= 0;


// Holds current counts for the encoder
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
 * Main loop
 */
void loop() {
  // Timeout if we haven't received a command int TIMEOUT millisecounds
  if (millis() - LastPiCommandTime >= MOTOR_TIMEOUT) {
      Motor1Power = 0;
      Motor2Power = 0;
  }
  
  
  // Read Encoders
  data.encoder1Count = Encoder1.readEncoder();
  data.encoder2Count = Encoder2.readEncoder();

  #ifdef DEBUG2
  if (Motor1Power != 0 && Motor2Power != 0) {
    Serial.print("Sending: ");
    Serial.print(Motor1Power);
    Serial.print(" | ");
    Serial.println(Motor2Power);
  }
  #endif
  
  // Send powers to motors
  ST.motor(1,Motor1Power);
  ST.motor(2,Motor2Power);
    
}

/**
 * Callback for receiving i2c data
 */
void receiveData(int byteCount){
  
  // Buffer to hold the commands read from the motor
  int8_t readBuffer[MOTOR_COMMAND_SIZE];
  uint8_t i = 0;
  
  while(Wire.available()) {
    // Makes sure we aren't getting junk from the i2c line 
    if (byteCount == MOTOR_COMMAND_SIZE) {
      // save the data into the buffer
      readBuffer[i++] = Wire.read();
    }
    else {
      // clear our buffer
      Wire.read();
    }
  }
  
  // make sure that we read the correct number of bytes and that we have a move command
  // and update motor powers
  if (i == MOTOR_COMMAND_SIZE && readBuffer[0] == MOTOR_COMMAND) {
    Motor1Power = readBuffer[1];
    Motor2Power = readBuffer[2];
  
    // update the time we last received a message
    LastPiCommandTime = millis();
  }
  
  #ifdef DEBUG
  if (Motor1Power != 0 && Motor2Power != 0) {
    Serial.print("Received ");
    Serial.print(byteCount);
    Serial.println(" bytes");
    Serial.print("Motor 1 Power: ");
    Serial.println(Motor1Power);
    Serial.print("Motor 2 Power: ");
    Serial.println(Motor2Power);
  }
  #endif
}


/**
 * Callback for i2c data request
 */
void sendData(){
  Wire.write((byte*)(&data),sizeof(EncoderData));
  
  #ifdef DEBUG
  Serial.print("Encoder 1 Count: ");
  Serial.println(data.encoder1Count);
  Serial.print("Encoder 2 Count: ");
  Serial.println(data.encoder2Count);
  #endif
}
