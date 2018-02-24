#include <Wire.h>
#include <SPI.h>
#include <Sabertooth.h>
#include <Encoder_Buffer.h>
#include <stdint.h>
#include <SoftwareSerial.h>

#define SLAVE_ADDRESS 0x04
#define BUFFER_SIZE 4
#define BAUD_RATE 115200

#define SABERTOOTH_ADDRESS 128
#define ENCODER1_SELECT_PIN 7
#define ENCODER2_SELECT_PIN 8

//#define DEBUG

// struct to send both encoder counts over SPI
typedef struct EncoderDataTag {
    signed long encoder1Count;
    signed long encoder2Count;
} EncoderData;

jjj
// Buffer for reading motor control commands
int8_t ReadBuffer[BUFFER_SIZE];

uint8_t NumBytes = 0;

int8_t Motor1Power = 0;
int8_t Motor2Power = 0;

// Holds current counts for the encoder
EncoderData data;

// initialize sabertooth and encoders
//Sabertooth ST(SABERTOOTH_ADDRESS);
Encoder_Buffer Encoder1(ENCODER1_SELECT_PIN);
Encoder_Buffer Encoder2(ENCODER2_SELECT_PIN);


SoftwareSerial SWSerial(NOT_A_PIN, 2); // RX on no pin (unused), TX on pin 11 (to S1).
Sabertooth ST(128, SWSerial); // Address 128, and use SWSerial as the serial port.

void setup() {
  
  // start Serial and SPI
  Serial.begin(BAUD_RATE);
  // SabertoothTXPinSerial.begin(BAUD_RATE);
  SWSerial.begin(BAUD_RATE);
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

    #ifdef DEBUG
    Serial.print("Motor 1 Power: ");
    Serial.println(Motor1Power);
    Serial.print("Motor 2 Power: ");
    Serial.println(Motor2Power);
    #endif
    
    ST.motor(1,Motor1Power);
    delay(20); 
    ST.motor(2,Motor2Power);
    
    delay(500);
}

// receive motor commands
void receiveData(int byteCount){
  #ifdef DEBUG
  Serial.print("Received ");
  Serial.print(byteCount);
  Serial.println(" bytes");
  
//  Serial.print("Data received: ");
  #endif
//  if (byteCount == BUFFER_SIZE) {
//    
//     while (Wire.available() != BUFFER_SIZE);
//     
//     if (Wire.read() == 'm') {
//       Motor1Power = Wire.read();
//       Motor2Power = Wire.read();
//       Wire.read();
//     }
//  }

  
    NumBytes = 0; 
    while(Wire.available()) {
      if (NumBytes == 0) {
        Wire.read();
      }
      else if (NumBytes == 1) {
        Motor1Power = Wire.read();
      }
      else if (NumBytes == 2) {
        Motor2Power = Wire.read();
      }
      else {
        Wire.read();
      }
      NumBytes++;
    }
}
//        ReadBuffer[NumBytes] = Wire.read();
//        
//        #ifdef DEBUG
//        Serial.print(ReadBuffer[NumBytes]);
//        Serial.print(" ");
//        #endif
//        
//        NumBytes++;
//      }
//      else {
//        if (ReadBuffer[0] == 'm') {
//            
//          Motor1Power = ReadBuffer[1];
//          Motor2Power = ReadBuffer[2];
//          
//          #ifdef DEBUG
//          Serial.println();
//          Serial.print("Moving: L ");
//          Serial.print(ReadBuffer[1]);
//          Serial.print("| R ");
//          Serial.println(ReadBuffer[2]);
//          #endif
//          NumBytes = 0;
//        }
//      }  
//    }
//
//
//}

// send encoder data
void sendData(){
Wire.write((byte*)(&data),sizeof(EncoderData));
}