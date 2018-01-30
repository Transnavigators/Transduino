#include <Sabertooth.h>
#include <SPI.h>
#include <util/crc16.h>

//Setup Sabertooth on address 128
Sabertooth ST(128);

//Instructions for RPi->Arduino serial communication
const byte STOP = 0x00;
const byte SETMAX = 0x10;
const byte GO = 0x20;
const byte CHECKV = 0x30;

//When to execute the command
const byte NOW = 0x00;
const byte LATER = 0x01;

const uint8_t VOLT_METER = A0;

//Motor encoder input pins
const uint8_t A_1 = digitalPinToInterrupt(2);
const uint8_t B_1 = 4;
//const uint8_t Z_1 = 3;
const uint8_t A_2 = digitalPinToInterrupt(3);
const uint8_t B_2 = 5;
//const uint8_t Z_2 = 2;

// Slave Select pins for encoders 1 and 2
// Feel free to reallocate these pins to best suit your circuit
const int slaveSelectEnc1 = 7;
const int slaveSelectEnc2 = 8;

// These hold the current encoder count.
signed long encoder1Count = 0;
signed long encoder2Count = 0;

//The measured max speed of the motors at power = 127 in m/s
const float FULL_SPEED = 1;

//The maximum difference between target velocity and current velocity
const float MAX_V_ERROR = 0.001;

//The maximum difference between target acceleration and current acceleration
const float MAX_A_ERROR = 0.001;

//Smoothing value when estimating new speeds
const float ALPHA = 0.8;

//The number of rotations to travel a meter
//(2*pi*R/Gear ratio)
//Let R = 3", Gear ratio = 30
//TODO: Replace with actual numbers
const float ROT_PER_M = 62.6594263365;

//The width between wheels
const float WIDTH = 0.762;
//The time in us before the arduino should adjust the motor power again
//(1/100kHz)
const unsigned long CHECK_DELAY = 100;

//Number of us to wait in between commands to try to get an exact distance
int del = 0;

//Turning angle
float angle = 0;

//Equivalent turning radius
float radius = 0;

//Global variables for target velocity and acceleration
long maxVel = 0;
long maxAccel = 0;
long maxCentripAccel = 0;

//Ratio in speed between center and right wheel
float C_to_R_Ratio = 1;

//Ratio in speed from the center to left wheel
float C_to_L_Ratio = 1;

int rotations1 = 0;
int rotations2 = 0;

long currVel1 = 0;
long currVel2 = 0;

long currAccel1 = 0;
long currAccel2 = 0;

//Stores the next instruction
int nextRotations1 = 0;
int nextRotations2 = 0;
float nextDelay = 0;
float nextAngle = 0;
float nextC_to_R_Ratio = 1;
float nextC_to_L_Ratio = 1;

//The last time the encoder's value was checked
unsigned long prev_time = 0;

//Current power of each motor
int power1 = 0;
int power2 = 0;

int turnPower = 0;
int drivePower = 0;

//The next time when the control feedback loop should run
unsigned long checkTime = 0;

//1 when motor is running forward, 0 when backward
volatile int cw1 = 1;
volatile int cw2 = 1;

struct DataPacket {
  uint8_t Address;
  uint8_t PacketID;
  long encoder1Count;
  long encoder2Count;
  long deltaTime;
  //int voltage;
  uint16_t CRC;
};

void setup() {
	//Encoder pins
	for(int i=2;i<=7;i++) {
		pinMode(i,INPUT);
	}
	//LED pins
	for(int i=8;i<=11;i++) {
		pinMode(i,OUTPUT);
	}
	
	//Set the ramping value which decides how fast the motor can change speeds (1-80, low is faster)
	ST.setRamping(5);

  //Initialize the encoders
  initEncoders();

  //Initialize serial port
	//Set the speed to the default Raspberry Pi/Lidar serial speed
	Serial.begin(115200);
	while (!Serial) {
		; //Wait for serial port to connect (if USB)
	}
}

uint16_t MakeCRC(struct DataPacket *InPacket)
{
 uint16_t TempCRC = 0;
 TempCRC = _crc16_update(TempCRC, InPacket->Address);
 TempCRC = _crc16_update(TempCRC, InPacket->PacketID);
 TempCRC = _crc16_update(TempCRC, InPacket->encoder1Count);
 TempCRC = _crc16_update(TempCRC, InPacket->encoder2Count);
 TempCRC = _crc16_update(TempCRC, InPacket->deltaTime);
 //TempCRC = _crc16_update(TempCRC, InPacket->voltage);
 return TempCRC;
}

void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(slaveSelectEnc1, OUTPUT);
  pinMode(slaveSelectEnc2, OUTPUT);
  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
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

void updateInstruction() {
	//Both sides reached their goal
	//Don't let the control loop send a new command for the delay to finish fractional turns
	checkTime = micros()+(del/currVel1);
	
	//Use the next instruction variables
	rotations1 = nextRotations1;
	rotations2 = nextRotations2;
	angle = nextAngle;
	C_to_L_Ratio = nextC_to_L_Ratio;
	C_to_R_Ratio = nextC_to_R_Ratio;
	
	//Stop the motor if the next instruction is to stop
	//Avoid floating point equality check
	if(rotations1 == 0 && rotations2 == 0 && abs(angle) < 0.0001) {
		ST.motor(1,0);
		ST.motor(2,0);
	}
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

void stopMoving(byte now) {
	nextRotations1 = 0;
	nextRotations2 = 0;
	nextC_to_R_Ratio = 1;
	nextC_to_L_Ratio = 1;
	nextAngle = 0;
	if(now) {
		ST.stop();
		rotations1 = 0;
		rotations2 = 0;
		C_to_R_Ratio = 1;
		C_to_L_Ratio = 1;
		angle = 0;
	}
}

void setMax(byte now) {
	float v = Serial.parseFloat();
	if(v != 0) {
		maxVel = v*1000000;
	}
	float a = Serial.parseFloat();
	if(a != 0) {
		maxAccel = a*1000000;
	}
	float ac = Serial.parseFloat();
	if(a != 0) {
		maxCentripAccel = ac*1000000;
	}
}

void go(byte now) {
	
	//Distance in m
	float dist = Serial.parseFloat();
	//Distance in units of revolutions of motor
	float numRevs = dist*ROT_PER_M;
	
	int rev = (int)round(numRevs);
	
	//Fractional number of revolutions
	//Delay by delay/speed before executing the next command
	float de = (1000000/ROT_PER_M)*(numRevs - rev);
	//Angle in radians
	angle = Serial.parseFloat();
	float offset = WIDTH*angle/2;
	radius = dist/angle;
	if(now) {
		del = de;
		C_to_R_Ratio = dist/(dist+offset);
		C_to_L_Ratio = dist/(dist-offset);
    rotations1 = rev*C_to_L_Ratio;
    rotations2 = rev*C_to_R_Ratio;
		
		//Don't wait
		checkTime = 0;
	}
	else {
		//Set the values for the next command after the current one is done
		nextDelay = de;
		nextC_to_R_Ratio = dist/(dist+offset);
		nextC_to_L_Ratio = dist/(dist-offset);
    nextRotations1 = rev*nextC_to_L_Ratio;
    nextRotations2 = rev*nextC_to_L_Ratio;
	}
}

void checkVoltage() {
	//The 10 corrects for the voltage divider, the 2 correct for having 2 batteries,
	//and each unit has a resolution of 5V/1024 
	float voltage = analogRead(VOLT_METER)*(2*10*5/1024.0);
	int numLEDs = 0;
	
	//Each LED represents 25% of charge
	//The voltages are from here:
	//https://www.energymatters.com.au/components/battery-voltage-discharge/
	if(voltage >= 12.8) {
		numLEDs = 4;
	}
	else if(voltage >= 12.6) {
		numLEDs = 3;
	}
	else if(voltage >= 12.30) {
		numLEDs = 2;
	}
	else if(voltage >= 12) {
		numLEDs = 1;
	}
	
	//Turn on the first n LEDs, turn off the rest
	for(int i=8;i<8+numLEDs;i++) {
		digitalWrite(i,1);
	}
	for(int i=8+numLEDs;i<=11;i++) {
		digitalWrite(i,0);
	}
}

void loop() {
	// put your main code here, to run repeatedly:
	if(Serial.available() > 0) {
		byte byteRead = Serial.read();
		//Wait for Arduino's address
		if(byteRead == 0xEE) {
			byteRead = Serial.read();
			byte now = byteRead & 0x0F;
			switch(byteRead & 0xF0) {
				case STOP: {
					//stop
					stopMoving(now);
				}
				break;
				case SETMAX: {
					//set max v,a,ac
					setMax(now);
				}
				break;
				case GO: {
					//set max v,a,ac
					go(now);
				}
				break;
				case CHECKV: {
					//check the battery voltage
					checkVoltage();
				}
				break;
				default:
				break;
			}
		}
	}
  unsigned long currTime = micros();

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
  
  //dp.voltage = analogRead(VOLT_METER);

  //Calculate a CRC for error correction
  dp.CRC = MakeCRC(&dp);

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
  if((encoder1Count & 0x40000000L) | (encoder2Count & 0x40000000L)) {
    //Reset the count to 0 if so
    clearEncoderCount();
    encoder1Count = 0;
    encoder2Count = 0;
  }
  //Subtract the number of rotations since the last check
  rotations1 = constrain(rotations1 - dp.encoder1Count,0,32767);
  rotations2 = constrain(rotations2 - dp.encoder2Count,0,32767);
  if(rotations1 <= 0 && rotations2 <= 0) {
    updateInstruction();
  }
    
	//Not stopped by delay or instruction
 //Angle = 0 and both rotation counters when we want to stop
 //If one side is done moving, update often
 //Otherwise, only check after the specified delay
	if(checkTime < currTime || rotations1 != 0 || rotations2 != 0 || (angle != 0 && rotations1 != 0 && rotations2 != 0)) {
		int dpower1 = 0;
		int dpower2 = 0;

    //Process the speed
    //TODO: use math to find currVel1 and 2 using real constants
    //number of rotations = dp.encoder1Count/1024
    //distance in meters = 62.6594263365*(dp.encoder1Count/1024)
    //distance in meters = 0.06119084603*dp.encoder1Count
    //dp.encoder1Count should only reach up to 10000 in one cycle, so normalize it
    //distance in um = 61190*dp.encoder1Count
    //speed in m/s = (61190/dp.deltaTime)*dp.encoder1Count
    //speed in m/s *10^6 = (61190846030/dp.deltaTime)*dp.encoder1Count
    //time in seconds = dp.deltaTime/1000000

    //Calculate new velocities
    long newCurrVel1 = (61190846030L/dp.deltaTime)*dp.encoder1Count;
    long newCurrVel2 = (61190846030L/dp.deltaTime)*dp.encoder2Count;
    
    //Calculate acceleration from velocity
    currAccel1 = 1000000*(newCurrVel1 - currVel1)/dp.deltaTime;
    currAccel2 = 1000000*(newCurrVel2 - currVel2)/dp.deltaTime;

    //Replace the old velocity values now that they aren't needed
    currVel1 = newCurrVel1;
    currVel2 = newCurrVel2;
    
		//Angle is off
		if(abs(currVel1-currVel2) > MAX_V_ERROR) {
			//Going to the right too much
			if(currVel1 > currVel2) {
				//Left is going too fast
				if(currVel1 > maxVel) {
					dpower1--;
				}
				//Right is too slow
				if(currVel2 < maxVel && abs(currVel2*currVel2/radius) < maxCentripAccel) {
					dpower2++;
				}
			}
			//Going to the left too much
			else {
				//Right is too fast
				if(currVel2 > maxVel) {
					dpower2--;
				}
				//Left is too slow
				if(currVel1 < maxVel && abs(currVel1*currVel1/radius) < maxCentripAccel) {
					dpower1++;
				}
			}
		}
		//Left speed is not max and accel is not above threshold
		if(abs(currVel1 - maxVel) > MAX_V_ERROR && abs(currAccel1) < maxAccel) {
			//left too slow
			if(currVel1 < maxVel) {
				dpower1++;
			}
			//left too fast
			else {
				dpower1--;
			}
		}
		//Right speed is not max and accel is not above threshold
		if(abs(currVel2 - maxVel) > MAX_V_ERROR && abs(currAccel2) < maxAccel) {
			//right too slow
			if(currVel2 < maxVel) {
				dpower2++;
			}
			//right too fast
			else {
				dpower2--;
			}
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
			checkTime = micros()+CHECK_DELAY;
		}
	}
}
