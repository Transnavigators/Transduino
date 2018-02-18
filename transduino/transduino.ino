#include <Sabertooth.h>
#include <SPI.h>
#include <Encoder_Buffer.h>
#include <FastCRC.h>

#define DEBUG 1

//Setup Sabertooth on address 128
Sabertooth ST(128);

FastCRC16 CRC16;

// Instructions for RPi->Arduino serial communication
const byte STOP = 0x00;
const byte GO = 0x20;

// Slave Select pins for encoders 1 and 2
// Feel free to reallocate these pins to best suit your circuit
Encoder_Buffer enc1(7);
Encoder_Buffer enc2(8);

// These hold the current encoder count.
signed long encoder1Count = 0;
signed long encoder2Count = 0;

//These hold the velocities we will try to move at
signed long targetVel1 = 0;
signed long targetVel2 = 0;

//The maximum difference between target velocity and current velocity
//TODO: Change this value after testing
const long MAX_V_ERROR = 100;

// How long to wait in us before stopping the wheelchair if commands stop coming
const long TIMEOUT = 250000;

// The time in us before the Arduino should adjust the motor power again
//TODO: Change this to stabilize the system
const unsigned long ZETA = 500;

//(1e6)*2Pi*(6/2)*0.0254/1024
const unsigned long UM_PER_PULSE = 467557344L;

//The current speed we want to match
long currVel1 = 0;
long currVel2 = 0;

//The last time the encoder's value was checked
unsigned long prevTime = 0;

//The last time the Pi sent a command
unsigned long lastCommandTime = 0;

//Current power of each motor
int power1 = 0;
int power2 = 0;

const int MAX_POWER = 128;
const int MIN_POWER = -128;

//The next time when the control feedback loop should run
unsigned long checkTime = 0;

struct OdomPacket {
  uint8_t Address;
  uint8_t PacketID;
  long encoder1Count;
  long encoder2Count;
  unsigned long deltaTime;
  uint16_t CRC;
};

//The packet we send
//Use global variable to speed up code 
OdomPacket op;

const uint16_t packetlen = sizeof(OdomPacket)-sizeof(uint16_t);

void setup() {
  
  SPI.begin();
  //Initialize the encoders
  enc1.initEncoder();
  enc2.initEncoder();
  
  //Initialize serial port
  //Set the speed to the default Raspberry Pi/LIDAR serial speed
  Serial.begin(115200);
  while (!Serial) {
    ; //Wait for serial port to connect (if USB)
  }
  ST.autobaud();
  //Set the ramping value which decides how fast the motor can change speeds (1-80, low is faster)
  ST.setRamping(5);
  
  //Every packet begins with 0xEE to let the Pi know when to start listening
  op.Address = 0xEE;
  
  //Packet number allows more than 1 packet format
  op.PacketID = 0x01;
}

//This is the main loop which reads serial commands
void loop() {
  unsigned long currTime = micros();
  if(Serial.available() > 0) {
    byte byteRead = Serial.read();
    //Wait for Arduino's address
    if(byteRead == 0xEE) {
      byteRead = Serial.read();
      switch(byteRead) {
        case STOP:
          ST.stop();
          targetVel1 = 0;
          targetVel2 = 0;
          #ifdef DEBUG
            Serial.println("I stopped moving.");
          #endif
          lastCommandTime = currTime;
          break;
        case GO:
          targetVel1 = Serial.parseInt();
          targetVel2 = Serial.parseInt();
          lastCommandTime = currTime;
          #ifdef DEBUG
            Serial.println("I received a new command to go at "+String(targetVel1)+"um/s and "+String(targetVel2)+"um/s after "+String(currTime-lastCommandTime)+" us.");
          #endif
          break;
        default:
          break;
      }
    }
  }
  //Check for timeout
  if(lastCommandTime + TIMEOUT > currTime) {
    ST.stop();
    targetVel1 = 0;
    targetVel2 = 0;
    #ifdef DEBUG
      Serial.println("I stopped moving due to a timeout.");
    #endif
  }
  else {
    //Find the number of times the encoder pulsed from the last check to now
    op.encoder1Count = enc1.readEncoder() - encoder1Count;
    op.encoder2Count = enc2.readEncoder() - encoder2Count;
  
    //Find the microseconds between the last speed check and now
    op.deltaTime = currTime - prevTime;
  
    //Calculate a CRC for error correction
    op.CRC = CRC16.ccitt((const uint8_t*)&op,packetlen);
  
    //Write the packet to the serial line
    Serial.write((byte*)&op,sizeof(OdomPacket));
    
    //Setup for next time
    encoder1Count += op.encoder1Count;
    encoder2Count += op.encoder2Count;
    prevTime = currTime;
    
    //Not stopped by delay or instruction
    //Otherwise, only check after the specified delay
    if(checkTime < currTime && (targetVel1 != 0 || targetVel2 != 0)) {
      
      //Calculate new velocities in um/s
      currVel1 = (op.encoder1Count*UM_PER_PULSE)/op.deltaTime;
      currVel2 = (op.encoder2Count*UM_PER_PULSE)/op.deltaTime;

      #ifdef DEBUG
        Serial.println("CurrVel1="+String(currVel1)+"um/s CurrVel2="+String(currVel2)+"um/s");
      #endif
      
      //Left wheel is too slow
      if(currVel1 < targetVel1 && targetVel1 - currVel1 > MAX_V_ERROR && power1 < MAX_POWER) {
        power1++;
        ST.motor(1,power1);
        checkTime = micros()+ZETA;
      }
      //Right wheel is too slow
      if(currVel2 < targetVel2 && targetVel2 - currVel2 > MAX_V_ERROR && power2 < MAX_POWER) {
        power2++;
        ST.motor(2,power2);
        checkTime = micros()+ZETA;
      }
      //Left wheel is too fast
      if(currVel1 > targetVel1 && currVel1 - targetVel1 > MAX_V_ERROR && power1 > MIN_POWER) {
        power1--;
        ST.motor(1,power1);
        checkTime = micros()+ZETA;
      }
      //Right wheel is too fast
      if(currVel2 > targetVel2 && currVel2 - targetVel2 > MAX_V_ERROR && power2 > MIN_POWER) {
        power2--;
        ST.motor(2,power2);
        checkTime = micros()+ZETA;
      }
    }
  
    //Check if any of the encoders are close to overflowing the long by testing the most significant bit
    //Shouldn't need this unless running for 1 week straight
    if((encoder1Count & 0x40000000L) || (encoder2Count & 0x40000000L)) {
      //Reset the count to 0 if so
      enc1.clearEncoderCount();
      enc2.clearEncoderCount();
      encoder1Count = 0;
      encoder2Count = 0;
    }
    
    #ifdef DEBUG
      Serial.println("Loop time: "+String(currTime-lastCommandTime)+" us. Encoder1="+String(encoder1Count)+" Encoder2="+String(encoder2Count));
    #endif
  }
  
}
