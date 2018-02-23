#include <Wire.h>
#include <SPI.h>
#include <Sabertooth.h>
#include <Encoder_Buffer.h>
#include <stdint.h>

#define SLAVE_ADDRESS 0x04
#define BUFFER_SIZE 4
#define BAUD_RATE 115200

#define SABERTOOTH_ADDRESS 128
#define ENCODER1_SELECT_PIN 7
#define ENCODER2_SELECT_PIN 8

#define DEBUG

// struct to send both encoder counts over SPI
typedef struct EncoderDataTag {
    signed long encoder1Count;
    signed long encoder2Count;
} EncoderData;


// Buffer for reading motor control commands
int8_t ReadBuffer[BUFFER_SIZE];

uint8_t NumBytes = 0;

// Holds current counts for the encoder
EncoderData data;

// initialize sabertooth and encoders
Sabertooth ST(SABERTOOTH_ADDRESS);
Encoder_Buffer Encoder1(ENCODER1_SELECT_PIN);
Encoder_Buffer Encoder2(ENCODER2_SELECT_PIN);

void setup() {
  
  // start Serial and SPI
  // Serial.begin(BAUD_RATE);
  SabertoothTXPinSerial.begin(BAUD_RATE);
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
    
    delay(500);
}

// receive motor commands
void receiveData(int byteCount){
  #ifdef DEBUG
  Serial.print("Received ");
  Serial.print(byteCount);
  Serial.println(" bytes");
  
  Serial.print("Data received: ");
  #endif
  
    while(Wire.available()) {
      if (NumBytes < BUFFER_SIZE) {
        ReadBuffer[NumBytes] = Wire.read();
        
        #ifdef DEBUG
        Serial.print(ReadBuffer[i]);
        #endif
        
        NumBytes++;
      }
      else {
        if (ReadBuffer[0] == 'm') {
            
          ST.motor(0,ReadBuffer[1]);
          ST.motor(1,ReadBuffer[2]);
          
          #ifdef DEBUG
          Serial.println()
          Serial.print("Moving: L ");
          Serial.print(ReadBuffer[1]);
          Serial.print("| R ");
          Serial.println(ReadBuffer[2]);
          #endif
        }   
      NumBytes = 0;
    }
  }


}

// send encoder data
void sendData(){
  Wire.write(Data,sizeof(EncoderData));
}
