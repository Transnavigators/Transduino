#include <Sabertooth.h>
//#include <EnableInterrupt.h>

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
const float ROT_PER_M = 62.6594263365;

//The width between wheels
const float WIDTH = 0.762;
//The time in us before the arduino should adjust the motor power again
//(1/100kHz)
const unsigned long CHECK_DELAY = 100;

//Number of rotations left before switching to the next command
volatile int rotations1 = 0;
volatile int rotations2 = 0;

//Number of us to wait in between commands to try to get an exact distance
int del = 0;

//Turning angle
float angle = 0;

//Equivalent turning radius
float radius = 0;

//Global variables for target velocity and acceleration
float maxVel = 0;
float maxAccel = 0;
float maxCentripAccel = 0;

//Ratio in speed between center and right wheel
float C_to_R_Ratio = 1;

//Ratio in speed from the center to left wheel
float C_to_L_Ratio = 1;

//Stores the next instruction
int nextRotations1 = 0;
int nextRotations2 = 0;
float nextDelay = 0;
float nextAngle = 0;
float nextC_to_R_Ratio = 1;
float nextC_to_L_Ratio = 1;

//PWM values from encoder
volatile int prev_time1 = 0;
volatile int prev_time2 = 0;

//Estimated current velocity and acceleration
volatile float currVel1 = 0;
volatile float currVel2 = 0;
volatile float currAccel1 = 0;
volatile float currAccel2 = 0;

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

void setup() {
	//Encoder pins
	for(int i=2;i<=7;i++) {
		pinMode(i,INPUT);
	}
	//LED pins
	for(int i=8;i<=11;i++) {
		pinMode(i,OUTPUT);
	}
	
	attachInterrupt(A_1, rising1, RISING);
	attachInterrupt(A_2, rising2, RISING);
	
	//Set the ramping value which decides how fast the motor can change speeds (1-80, low is faster)
	ST.setRamping(5);
	
	//Set the speed to the default Raspberry Pi/Lidar serial speed
	Serial.begin(115200);
	while (!Serial) {
		; //Wait for serial port to connect (if USB)
	}
}

void updateInstruction() {
	//Both sides reached their goal
	if(rotations1 <= 0 && rotations2 <= 0) {
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
}

void rising1() {
	int pwm_period = micros()-prev_time1;
	
	//If B is low when A rises, B is out of phase with A by 90 degrees to the right and the motor is clockwise
	//If B is high when A rises, B is out of phase with A by 90 degrees to the left and the motor is counter clockwise
	cw1 = digitalRead(B_1);
	//Calculate the equivalent velocity in m/s at the center
	//(1 rotation / [pwm_period] s)*(1000000us / s)*(1m / [ROT_PER_M] rotation)*(1 m/s / C_to_L_Ratio m/s)
	if(cw1 == 1) {
		float nextVel = (1000000/ROT_PER_M)/(pwm_period*C_to_L_Ratio);
	}
	else {
		float nextVel = -(1000000/ROT_PER_M)/(pwm_period*C_to_L_Ratio);
	}
	//Estimate current acceleration & velocity
	currAccel1 = ALPHA*currAccel1 + (1-ALPHA)*1000000*((nextVel-currVel1)/pwm_period);
	currVel1 = ALPHA*currVel1 + (1-ALPHA)*nextVel;
	
	rotations1--;
	updateInstruction();
	prev_time1 = prev_time1+pwm_period;
}

void rising2() {
	int pwm_period = micros()-prev_time2;
	
	//If B is low when A rises, B is out of phase with A by 90 degrees to the right and the motor is clockwise
	//If B is high when A rises, B is out of phase with A by 90 degrees to the left and the motor is counter clockwise
	cw2 = digitalRead(B_2);
	//Calculate the equivalent velocity in m/s at the center
	//(1 rotation / [pwm_period] s)*(1000000us / s)*(1m / [ROT_PER_M] rotation)*(1 m/s / C_to_R_Ratio m/s)
	if(cw2 == 1) {
		float nextVel = (1000000/ROT_PER_M)/(pwm_period*C_to_R_Ratio);
	}
	else {
		float nextVel = -(1000000/ROT_PER_M)/(pwm_period*C_to_R_Ratio);
	}
	
	//Estimate current acceleration & velocity
	currAccel2 = ALPHA*currAccel2 + (1-ALPHA)*1000000*((nextVel-currVel2)/(pwm_period));
	currVel2 = ALPHA*currVel2 + (1-ALPHA)*nextVel;
	
	rotations2--;
	updateInstruction();
	prev_time2 = prev_time2+pwm_period;
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
		maxVel = v;
	}
	float a = Serial.parseFloat();
	if(a != 0) {
		maxAccel = a;
	}
	float ac = Serial.parseFloat();
	if(a != 0) {
		maxCentripAccel = ac;
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
	//Not stopped by delay or instruction
	if(checkTime < currTime || rotations1 != 0 || rotations2 != 0 || (angle != 0 && rotations1 == 0 && rotations2 == 0)) {
		int dpower1 = 0;
		int dpower2 = 0;
		//int dDrivePower = 0;
		//int dTurningPower = 0;
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
				//dTurningPower--;
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
				//dTurningPower++;
			}
		}
		//left speed is not max and accel is not above threshold
		if(abs(currVel1 - maxVel) > MAX_V_ERROR && abs(currAccel1) < maxAccel) {
			//left too slow
			if(currVel1 < maxVel) {
				dpower1++;
				//dDrivePower++;
			}
			//left too fast
			else {
				dpower1--;
				//dDrivePower--;
			}
		}
		//Right speed is not max and accel is not above threshold
		if(abs(currVel2 - maxVel) > MAX_V_ERROR && abs(currAccel2) < maxAccel) {
			//right too slow
			if(currVel2 < maxVel) {
				dpower2++;
				//dDrivePower++;
			}
			//right too fast
			else {
				dpower2--;
				//dDrivePower--;
			}
		}
		//Cap power to range between -127 and 127
		power1 = constrain(power1+dpower1,-127,127);
		power2 = constrain(power2+dpower2,-127,127);
		//turnPower = constrain(turnPower+dTurningPower,-127,127);
		//drivePower = constrain(drivePower+dDrivePower,-127,127);
		
		//Send new command to motor if there is a change
		if(dpower1 != 0) {
			ST.motor(1,power1);
		}
		if(dpower2 != 0) {
			ST.motor(2,power2);
		}
		/*if(dTurningPower != 0) {
			ST.turn(turnPower);
		}
		if(dDrivePower != =) {
			ST.drive(drivePower);
		}
		*/
		
		//If there is a change, wait for the motor to respond before adjusting again
		if(dpower1 != 0 || dpower2 != 0) {
			checkTime = micros()+CHECK_DELAY;
		}
	}
}
