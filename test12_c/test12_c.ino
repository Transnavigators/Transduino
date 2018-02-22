#include <Wire.h>
#include <Sabertooth.h>

#define SLAVE_ADDRESS 0x04
#define BUFFER_SIZE 4


int numBytes = 0;
int readBuffer[BUFFER_SIZE];

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

// send commands to motor
void receiveData(int byteCount){
  Serial.print("Received ");
  Serial.print(byteCount);
  Serial.println(" bytes");
  
  while(Wire.available()) {
    if (byteCount < BUFFER_SIZE) {
      readBuffer[byteCount] = Wire.read();
	  
      Serial.print("Data received: ");
      Serial.println(readBuffer[byteCount]);
      byteCount++;
  	}
  	else {
      if (readBuffer[0] == 'm') {
        
        Serial.print("Moving: L ");
        Serial.print(readBuffer[1]);
        Serial.print("| R ");
        Serial.println(readBuffer[2]);
        
        ST.motor(0,readBuffer[1]);
        ST.motor(1,readBuffer[2]);
      }
	    byteCount = 0;
	  }
  }
}

// get encoder data
void sendData(){
  Wire.write(numBytes);
}
