#include <Sabertooth.h>
#include <SPI.h>
#include <Encoder_Buffer.h>
#include <util/crc16.h>

Sabertooth ST(128);

// Slave Select pins for encoders 1 and 2
// Feel free to reallocate these pins to best suit your circuit
Encoder_Buffer enc1(7);
Encoder_Buffer enc2(8);

// These hold the current encoder count.
signed long deltaEncoder1Count = 0;
signed long deltaEncoder2Count = 0;
signed long encoder1Count = 0;
signed long encoder2Count = 0;

//These hold the velocities we will try to move at
//2.2m/s or 5mph = top speed
signed long targetVel1 = 2.2e6;
signed long targetVel2 = 2.2e6;

//The maximum difference between target velocity and current velocity
//TODO: Change this value after testing
const long MAX_V_ERROR = 2e5;

long ZETA = 5e6;

//How long to wait in us before stopping the wheelchair if commands stop coming
const long TIMEOUT = 2500000;

//(1e6)*2Pi*(6/2)*0.0254/1024
const unsigned long UM_PER_PULSE = 61190846030L;

long currVel1 = 0;
long currVel2 = 0;

//The last time the encoder's value was checked
unsigned long prevTime = 0;

//The last time the Pi sent a command
unsigned long lastCommandTime = 0;

//Current power of each motor
int power1 = 0;
int power2 = 0;

int dpower1 = 0;
int dpower2 = 0;

unsigned long umPerPulseS = 0;

//The next time when the control feedback loop should run
unsigned long checkTime = 0;

boolean leftSlow = false;
boolean rightSlow = false;
unsigned long numOscillationsLeft = 0;
unsigned long numOscillationsRight = 0;

//The number of pulses to move forward before we have to stop
//2 meters
const unsigned long MAX_PULSES = 2139*2;

boolean reachedVel = false;

void setup() {
  //Set the ramping value which decides how fast the motor can change speeds (1-80, low is faster)
  ST.setRamping(5);

  //Initialize the encoders
  enc1.initEncoder();
  enc2.initEncoder();
  
  //Initialize serial port
  //Set the speed to the default Raspberry Pi/LIDAR serial speed
  Serial.begin(115200);
  while (!Serial) {
    ; //Wait for serial port to connect (if USB)
  }

  ZETA = findZeta();
  long maxVelError = findMaxVelError();
  Serial.println("Found optimal ZETA="+String(ZETA)+" and optimal MAX_V_ERROR="+String(maxVelError));
}

long findMaxVelError() {
  long increment = 1e5;
  long maxVelErr = 2e5;

  //Do a binary search for the best velocity error in 54 tests
  while(increment != 0) {
    long low = testVelError(maxVelErr-increment);
    long med = testVelError(maxVelErr);
    long high = testVelError(maxVelErr+increment);
    if(low < high && high < med) {
      maxVelErr = maxVelErr - increment;
    }
    else if(high < low && high < med) {
      maxVelErr = maxVelErr + increment;
    }
    increment = increment/2;
  }
  return maxVelErr;
}

long testVelError(int vError) {
  encoder1Count = 0;
  encoder2Count = 0;
  unsigned long totalVelErr1 = 0;
  unsigned long totalVelErr2 = 0;
  while(encoder1Count < MAX_PULSES || encoder2Count < MAX_PULSES) {
    unsigned long currTime = micros();
  
    //Find the number of times the encoder pulsed from the last check to now
    deltaEncoder1Count = enc1.readEncoder() - encoder1Count;
    deltaEncoder2Count = enc2.readEncoder() - encoder2Count;
  
    //Setup for next time
    encoder1Count += deltaEncoder1Count;
    encoder2Count += deltaEncoder2Count;
    
    //Not stopped by delay or instruction
    //Angle = 0 and both rotation counters when we want to stop
    //If one side is done moving, update often
    //Otherwise, only check after the specified delay
    if(checkTime < currTime && (targetVel1 != 0 || targetVel2 != 0)) {
      
      //Calculate new velocities in um/s
      umPerPulseS = UM_PER_PULSE/(currTime - prevTime);
      currVel1 = umPerPulseS*deltaEncoder1Count;
      currVel2 = umPerPulseS*deltaEncoder2Count;
  
      //Count number of times switch from incrementing to decrementing
      //Left wheel is too slow
      if(currVel1 < targetVel1 && targetVel1 - currVel1 > vError) {
        dpower1++;
      }
      //Right wheel is too slow
      if(currVel2 < targetVel2 && targetVel2 - currVel2 > vError) {
        dpower2++;
      }
      //Left wheel is too fast
      if(currVel1 > targetVel1 && currVel1 - targetVel1 > vError) {
        dpower1--;
      }
      //Right wheel is too fast
      if(currVel2 > targetVel2 && currVel2 - targetVel2 > vError) {
        dpower2--;
      }
      if(dpower1 == 0 && dpower2 == 0) {
        reachedVel = true;
      }
      if(reachedVel) {
        totalVelErr1 += currVel1 - targetVel1;
        totalVelErr2 += currVel2 - targetVel2;
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
      prevTime = currTime;

      //Print diagnostic info to console
      Serial.println("Loop time = "+String(currTime-lastCommandTime)+",power1 = "+String(power1)+" power2 = "+String(power2)+", delta vel1 = "+String(currVel1 - targetVel1)+", delta vel2 = "+String(currVel2 - targetVel2));
    }
  }
  //The next test will be done backwards
  targetVel1 = -targetVel1;
  targetVel2 = -targetVel2;

  //If the velocity was never reached to the specified error, this is a bad vError
  if(!reachedVel) {
    return 2147483647L;
  }
  return totalVelErr1+totalVelErr2;
}

long findZeta() {
  long increment = 2.5e6;
  long zeta = 5e6;

  //Do a binary search for the best zeta in 44 tests
  while(increment != 0) {
    long low = testZeta(zeta-increment);
    long med = testZeta(zeta);
    long high = testZeta(zeta+increment);
    if(low < high && high < med) {
      zeta = zeta - increment;
    }
    else if(high < low && high < med) {
      zeta = zeta + increment;
    }
    increment = increment/2;
  }
  return zeta;
}

long testZeta(int zeta) {
  encoder1Count = 0;
  encoder2Count = 0;
  while(encoder1Count < MAX_PULSES || encoder2Count < MAX_PULSES) {
    unsigned long currTime = micros();
  
    //Find the number of times the encoder pulsed from the last check to now
    deltaEncoder1Count = enc1.readEncoder() - encoder1Count;
    deltaEncoder2Count = enc2.readEncoder() - encoder2Count;
  
    //Setup for next time
    encoder1Count += deltaEncoder1Count;
    encoder2Count += deltaEncoder2Count;
    
    //Not stopped by delay or instruction
    //Angle = 0 and both rotation counters when we want to stop
    //If one side is done moving, update often
    //Otherwise, only check after the specified delay
    if(checkTime < currTime && (targetVel1 != 0 || targetVel2 != 0)) {
      
      //Calculate new velocities in um/s
      umPerPulseS = UM_PER_PULSE/(currTime - prevTime);
      currVel1 = umPerPulseS*deltaEncoder1Count;
      currVel2 = umPerPulseS*deltaEncoder2Count;
  
      //Count number of times switch from incrementing to decrementing
      //Left wheel is too slow
      if(currVel1 < targetVel1 && targetVel1 - currVel1 > MAX_V_ERROR) {
        dpower1++;
        if(!leftSlow) {
          numOscillationsLeft++;
          leftSlow = true;
        }
      }
      //Right wheel is too slow
      if(currVel2 < targetVel2 && targetVel2 - currVel2 > MAX_V_ERROR) {
        dpower2++;
        if(!rightSlow) {
          numOscillationsRight++;
          rightSlow = true;
        }
      }
      //Left wheel is too fast
      if(currVel1 > targetVel1 && currVel1 - targetVel1 > MAX_V_ERROR) {
        dpower1--;
        if(leftSlow) {
          numOscillationsLeft++;
          leftSlow = false;
        }
      }
      //Right wheel is too fast
      if(currVel2 > targetVel2 && currVel2 - targetVel2 > MAX_V_ERROR) {
        dpower2--;
        if(rightSlow) {
          numOscillationsRight++;
          rightSlow = false;
        }
      }
      if(dpower1 == 0 && dpower2 == 0) {
        reachedVel = true;
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
        checkTime = micros()+zeta;
      }
      prevTime = currTime;

      //Print diagnostic info to console
      Serial.println("Loop time = "+String(currTime-lastCommandTime)+",power1 = "+String(power1)+" power2 = "+String(power2)+", delta vel1 = "+String(currVel1 - targetVel1)+", delta vel2 = "+String(currVel2 - targetVel2));
    }
  }
  Serial.println("At zeta = "+String(zeta)+" Oscillated left "+String(numOscillationsLeft)+"Oscillated right "+String(numOscillationsRight));

  //The next test will be done backwards
  targetVel1 = -targetVel1;
  targetVel2 = -targetVel2;
  //If the velocity was never reached, this is a bad zeta
  if(!reachedVel) {
    return 2147483647L;
  }
  return numOscillationsLeft + numOscillationsRight;
}

void loop() {
}
