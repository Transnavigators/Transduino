#include <Sabertooth.h>
#include <SPI.h>
#include <util/crc16.h>
//Setup Sabertooth on address 128
Sabertooth ST(128);

//Instructions for RPi->Arduino serial communication
const byte STOP = 0x00;
const byte GO = 0x20;

// Slave Select pins for encoders 1 and 2
// Feel free to reallocate these pins to best suit your circuit
const int slaveSelectEnc1 = 7;
const int slaveSelectEnc2 = 8;

// These hold the current encoder count.
signed long encoder1Count = 0;
signed long encoder2Count = 0;

//These hold the velocities we will try to move at
signed long targetVel1 = 0;
signed long targetVel2 = 0;

//The maximum difference between target velocity and current velocity
//TODO: Change this value after testing
const long MAX_V_ERROR = 100;

//How long to wait in us before stopping the wheelchair if commands stop coming
const long TIMEOUT = 2500000;

//The time in us before the Arduino should adjust the motor power again
//(1/100kHz)
//TODO: Change this to stabilize the system
const unsigned long ZETA = 100;

//(1e6)*2Pi*(6/2)*0.0254/1024
const long UM_PER_PULSE = 61190846030L;

long currVel1 = 0;
long currVel2 = 0;

//The last time the encoder's value was checked
unsigned long prev_time = 0;

//The last time the Pi sent a command
unsigned long last_command_time = 0;

//Current power of each motor
int power1 = 0;
int power2 = 0;

int dpower1 = 0;
int dpower2 = 0;

//The next time when the control feedback loop should run
unsigned long checkTime = 0;

struct DataPacket {
  uint8_t Address;
  uint8_t PacketID;
  long encoder1Count;
  long encoder2Count;
  long deltaTime;
  uint16_t CRC;
};

void setup() {
  //Set the ramping value which decides how fast the motor can change speeds (1-80, low is faster)
  ST.setRamping(5);

  //Initialize the encoders
  initEncoders();

  //Initialize serial port
  //Set the speed to the default Raspberry Pi/LIDAR serial speed
  Serial.begin(115200);
  while (!Serial) {
    ; //Wait for serial port to connect (if USB)
  }
}

void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(slaveSelectEnc1, OUTPUT);
  pinMode(slaveSelectEnc2, OUTPUT);
  
  // Raise select pins
  // Communication begins when you drop the individual select signal
  digitalWrite(slaveSelectEnc1,HIGH);
  digitalWrite(slaveSelectEnc2,HIGH);
  
  SPI.begin();
  
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc1,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc1,HIGH);       // Terminate SPI conversation

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc2,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc2,HIGH);       // Terminate SPI conversation
}

long readEncoder(int encoder) {
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;
  
  // Read encoder 1
  if (encoder == 1) {
    digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);
    count_3 = SPI.transfer(0x00);
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation
  }
  
  // Read encoder 2
  else if (encoder == 2) {
    digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                      // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);
    count_3 = SPI.transfer(0x00);
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation
  }
  
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  
  return count_value;
}
void clearEncoderCount() {
  
  // Set encoder1's data register to 0
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
  // Write to DTR
  SPI.transfer(0x98);
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
  SPI.transfer(0xE0);
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation
  
  // Set encoder2's data register to 0
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation
  // Write to DTR
  SPI.transfer(0x98);
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder2's current data register to center
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation
  SPI.transfer(0xE0);
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation
}

//This is the main loop which reads serial commands
void loop() {
  unsigned long currTime = micros();
  if(Serial.available() > 0) {
    byte byteRead = Serial.read();
    //Wait for Arduino's address
    if(byteRead == 0xEE) {
      byteRead = Serial.read();
      byte now = byteRead & 0x0F;
      switch(byteRead & 0xF0) {
        case STOP:
          ST.stop();
          targetVel1 = 0;
          targetVel2 = 0;
          Serial.println("I stopped moving.");
          last_command_time = currTime;
          break;
        case GO:
          targetVel1 = Serial.parseInt();
          targetVel2 = Serial.parseInt();
          Serial.println("I received a new command after "+(currTime-last_command_time)+" us.");
          last_command_time = currTime;
          break;
        default:
          break;
      }
    }
  }
  
  if(last_command_time + TIMEOUT > currTime) {
    ST.stop();
    targetVel1 = 0;
    targetVel2 = 0;
    Serial.println("I stopped moving due to a timeout.");
  }

  //Check the encoders and send the speed to the Pi
  DataPacket dp;
  
  //Every packet begins with 0xEE to let the Pi know when to start listening
  dp.Address = 0xEE;
  
  //Packet number allows more than 1 packet format
  dp.PacketID = 0x01;

  //Find the number of times the encoder pulsed from the last check to now
  dp.encoder1Count = readEncoder(1) - encoder1Count;
  dp.encoder2Count = readEncoder(2) - encoder2Count;

  //Find the microseconds between the last speed check and now
  dp.deltaTime = currTime - prev_time;

  //Calculate a CRC for error correction
  dp.CRC = _crc16_update(dp.CRC, dp.Address);
  dp.CRC = _crc16_update(dp.CRC, dp.PacketID);
  dp.CRC = _crc16_update(dp.CRC, dp.encoder1Count);
  dp.CRC = _crc16_update(dp.CRC, dp.encoder2Count);
  dp.CRC = _crc16_update(dp.CRC, dp.deltaTime);

  //Write the packet to the serial line
  const byte* p = (const byte*) &dp;
  for(int i = 0; i < sizeof dp; i++) {
    Serial.write(*p++);
  }
  
  //Setup for next time
  encoder1Count += dp.encoder1Count;
  encoder2Count += dp.encoder2Count;
  prev_time = currTime;

  //Check if any of the encoders are close to overflowing the long by testing the most significant bit
  //Shouldn't need this unless running for 1 week straight
  if((encoder1Count & 0x40000000L) | (encoder2Count & 0x40000000L)) {
    //Reset the count to 0 if so
    clearEncoderCount();
    encoder1Count = 0;
    encoder2Count = 0;
  }
  
  //Not stopped by delay or instruction
  //Angle = 0 and both rotation counters when we want to stop
  //If one side is done moving, update often
  //Otherwise, only check after the specified delay
  if(checkTime < currTime && (targetVel1 !=0 || targetVel2 != 0)) {
    
    //Calculate new velocities in um/s
    currVel1 = (UM_PER_PULSE/dp.deltaTime)*dp.encoder1Count;
    currVel2 = (UM_PER_PULSE/dp.deltaTime)*dp.encoder2Count;
    
    //Left wheel is too slow
    if(currVel1 < targetVel1 && targetVel1 - currVel1 > MAX_V_ERROR) {
      dpower1++;
    }
    //Right wheel is too slow
    if(currVel2 < targetVel2 && targetVel2 - currVel2 > MAX_V_ERROR) {
      dpower2++;
    }
    //Left wheel is too fast
    if(currVel1 > targetVel1 && currVel1 - targetVel1 > MAX_V_ERROR) {
      dpower1--;
    }
    //Right wheel is too fast
    if(currVel2 > targetVel2 && currVel2 - targetVel2 > MAX_V_ERROR) {
      dpower2--;
    }
    
    //Cap power to range between -127 and 127
    power1 = constrain(power1+dpower1,-127,127);
    power2 = constrain(power2+dpower2,-127,127);
    
    //Send new command to motor if there is a change
    if(dpower1 != 0) {
      ST.motor(1,power1);
    }
    if(dpower2 != 0) {
      ST.motor(2,power2);
    }
    
    //If there is a change, wait for the motor to respond before adjusting again
    if(dpower1 != 0 || dpower2 != 0) {
      checkTime = micros()+ZETA;
    }
  }
}
