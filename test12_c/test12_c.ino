#include <Wire.h>
#include <Sabertooth.h>

#define SLAVE_ADDRESS 0x04
int numBytes = 0;
int number1 = 0;
int number2 = 0;
int state = 0;

//Setup Sabertooth on address 128
Sabertooth ST(128);


void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(115200); // start serial for output
  SabertoothTXPinSerial.begin(115200);
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  Serial.println("Ready!");
}
void loop() {
  delay(100);
}

// callback for received data
void receiveData(int byteCount){
  while(Wire.available()) {
    if (numBytes == 0) {
      number1 = Wire.read();
      Serial.print("data received m1: ");
      Serial.println(number1);
 
    }
    else if (numBytes == 1) {
      number2 = Wire.read();
      Serial.print("data received m2: ");
      Serial.println(number2);
      ST.motor(1, number1);
      ST.motor(2, number2);
      numBytes = 0;
  }
}

// callback for sending data
void sendData(){
  Wire.write(number*number);
}
