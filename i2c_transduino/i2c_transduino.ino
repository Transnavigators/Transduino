#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <Encoder_Buffer.h>
#include <stdint.h>

#define SLAVE_ADDRESS 0x04
#define MOTOR_COMMAND_SIZE 3
#define MOTOR_COMMAND 'm'
#define BAUD_RATE 115200

#define SABERTOOTH_ADDRESS 128
#define ENCODER1_SELECT_PIN 7
#define ENCODER2_SELECT_PIN 8
#define SW_SERIAL_PORT 2

#define DEBUG

// struct to send both encoder counts over SPI
typedef struct EncoderDataTag {
    signed long encoder1Count;
    signed long encoder2Count;
} EncoderData;

int8_t Motor1Power = 0;
int8_t Motor2Power = 0;

// Holds current counts for the encoder
EncoderData data;

// initialize sabertooth and encoders

#ifndef DEBUG
Sabertooth ST(SABERTOOTH_ADDRESS);
#endif
#ifdef DEBUG
SoftwareSerial SWSerial(NOT_A_PIN, SW_SERIAL_PORT); // RX on no pin (unused), TX on pin 11 (to S1).
Sabertooth ST(SABERTOOTH_ADDRESS, SWSerial); // Address 128, and use SWSerial as the serial port.
#endif

Encoder_Buffer Encoder1(ENCODER1_SELECT_PIN);
Encoder_Buffer Encoder2(ENCODER2_SELECT_PIN);

void setup() {
  
  // start Serial
  #ifdef DEBUG
  SWSerial.begin(BAUD_RATE);
  #endif
  
  Serial.begin(BAUD_RATE);
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

void loop() {
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
    
    ST.motor(1,Motor1Power);
    ST.motor(2,Motor2Power);
    
}

// receive motor commands
void receiveData(int byteCount){
  #ifdef DEBUG2
  Serial.print("Received ");
  Serial.print(byteCount);
  Serial.println(" bytes");
  #endif
  
  int8_t readBuffer[MOTOR_COMMAND_SIZE];
  int i = 0;
  while(Wire.available()) {
    if (byteCount == MOTOR_COMMAND_SIZE) {
      readBuffer[i++] = Wire.read();
    }
    else {
      Wire.read();
    }
  }
  if (i == MOTOR_COMMAND_SIZE && readBuffer[0] == MOTOR_COMMAND) {
    Motor1Power = readBuffer[1];
    Motor2Power = readBuffer[2];
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


// send encoder data
void sendData(){
  Wire.write((byte*)(&data),sizeof(EncoderData));
  #ifdef DEBUG
  Serial.print("Encoder 1 Count: ");
  Serial.println(data.encoder1Count);
  Serial.print("Encoder 2 Count: ");
  Serial.println(data.encoder2Count);
  #endif
}
