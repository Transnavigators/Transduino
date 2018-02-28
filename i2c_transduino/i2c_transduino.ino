#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <Encoder_Buffer.h>
#include <stdint.h>

#define SLAVE_ADDRESS 0x04
#define BUFFER_SIZE 4
#define BAUD_RATE 115200

#define SABERTOOTH_ADDRESS 128
#define ENCODER1_SELECT_PIN 7
#define ENCODER2_SELECT_PIN 8
#define SW_SERIAL_PORT 2

#define TIMEOUT 10000

#define DEBUG

// struct to send both encoder counts over SPI
typedef struct EncoderDataTag {
    signed long encoder1Count;
    signed long encoder2Count;
} EncoderData;

// Buffer for reading motor control commands
int8_t ReadBuffer[BUFFER_SIZE];

uint8_t NumBytes = 0;

int8_t Motor1Power = 0;
int8_t Motor2Power = 0;

long lastCommand = 0;
boolean newCommand = false;
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
    //Timeout after 1ms
    if(!newCommand && (micros() > lastCommand+TIMEOUT)) {
      ST.stop();
    }
    else {
      newCommand = false;
      if(Motor1Power == 0 && Motor2Power == 0) {
        ST.stop();
      }
      if(Motor1Power != 0) {
        ST.motor(1,Motor1Power);
      }
      if(Motor2Power != 0) {
        ST.motor(2,Motor2Power);
      }
    }
}

// receive motor commands
void receiveData(int byteCount){
  #ifdef DEBUG
  Serial.print("Received ");
  Serial.print(byteCount);
  Serial.println(" bytes");
  #endif

  
  
  NumBytes = 0; 
  while(Wire.available()) {
    switch (NumBytes) {
      case 0: {
        Wire.read();
        break;
      }
      case 1: {
        Motor1Power = Wire.read();
        break;
      }
      case 2: {
        Motor2Power = Wire.read();
        break;
      }
      default: {
        Wire.read();
      }
    }
    NumBytes++;
  }
  lastCommand = micros();
  newCommand = true;
  #ifdef DEBUG
  if (NumBytes==3) {
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
