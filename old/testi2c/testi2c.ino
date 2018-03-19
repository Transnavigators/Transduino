
#include <Wire.h>
#include <stdint.h>

#define SLAVE_ADDRESS 0x04
#define BAUD_RATE 115200


#define DEBUG

int8_t Motor1Power = 0;

void setup() {
  
  // start Serial and SPI
  Serial.begin(BAUD_RATE);
  
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
    delay(500);
}

// receive motor commands
void receiveData(int byteCount){
  #ifdef DEBUG
  Serial.print("Received ");
  Serial.print(byteCount);
  Serial.print(" bytes: ");
  #endif
    while(Wire.available()) {
      Motor1Power = Wire.read();
      Serial.print(Motor1Power);
      Serial.print(",");
    }
    Serial.println();
}

// send encoder data
void sendData(){
  Wire.write((byte*)(&Motor1Power),sizeof(Motor1Power));
}
