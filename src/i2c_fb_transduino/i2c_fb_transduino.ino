#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <Encoder_Buffer.h>
#include <stdint.h>

#define SLAVE_ADDRESS 0x04
#define BAUD_RATE 115200

#define SABERTOOTH_ADDRESS 128
#define ENCODER1_SELECT_PIN 7
#define ENCODER2_SELECT_PIN 8
#define SW_SERIAL_PORT 2


#define MOTOR_COMMAND_SIZE 9
#define MOTOR_COMMAND 'm'
#define MOTOR_TIMEOUT 1000

//2^8 * 10^6 * 2 * pi * (6 / 2) * 0.0254 / 4096
#define PULSE_TO_UM 29924
//12 / 2^8 = 0.05m/s
#define MAX_ERR 12
/**
 * Defining DEBUG switches to using software serial
 * for communication with the Sabertooth Motor Controller
 * 
 * Connect S1 of the Sabertooth to Pin SW_SERIAL_PORT in debug mode
 * Connect S1 of the Sabertooth to Pin 1 when not in debug mode
 */
#define DEBUG 0

// struct to send both encoder counts over SPI
typedef struct EncoderDataTag {
    signed long encoder1Count;
    signed long encoder2Count;
} EncoderData;

// Power of Motor1 and Motor2
int8_t Motor1Power = 0;
int8_t Motor2Power = 0;

// The last time we received a messge from the arduino
// used to timeout the arduino after MOTOR_TIMEOUT milliseconds
long LastPiCommandTime= 0;

long lastLoopTime = 0;
long deltaTime = 0;
long now = 0;
long tempEncoderCount1 = 0;
long tempEncoderCount2 = 0;
long encoderSpeed1 = 0;
long encoderSpeed2 = 0;
long dSpeed1 = 0;
long dSpeed2 = 0;
// Holds current counts for the encoder
EncoderData data;

// initialize sabertooth
#if DEBUG
SoftwareSerial SWSerial(NOT_A_PIN, SW_SERIAL_PORT);
Sabertooth ST(SABERTOOTH_ADDRESS, SWSerial);
#else
Sabertooth ST(SABERTOOTH_ADDRESS);
#endif

// initialize the encoders
Encoder_Buffer Encoder1(ENCODER1_SELECT_PIN);
Encoder_Buffer Encoder2(ENCODER2_SELECT_PIN);


/**
 * Setup routine
 */
void setup() {
  
  // start Serial
  #if DEBUG
  SWSerial.begin(BAUD_RATE);
  #endif
  Serial.begin(BAUD_RATE);
  
  // start SPI
  SPI.begin();
  
  // Initialize encoders
  Encoder1.initEncoder();
  Encoder2.initEncoder();
  
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  #if DEBUG
  Serial.println("Ready!");
  #endif
}

/**
 * Main loop
 */
void loop() {
  now = millis();
  // Timeout if we haven't received a command int TIMEOUT millisecounds
  if (now - LastPiCommandTime >= MOTOR_TIMEOUT) {
      Motor1Power = 0;
      Motor2Power = 0;
  }
  deltaTime = now-lastLoopTime;
  
  // Read Encoders
  tempEncoderCount1 = Encoder1.readEncoder();
  tempEncoderCount2 = Encoder2.readEncoder();

  //Find difference from last values
  //Calculate speed in m/s in Q8.23 format
  encoderSpeed1 = (tempEncoderCount1 - data.encoder1Count)*PULSE_TO_UM/deltaTime;
  encoderSpeed2 = (tempEncoderCount2 - data.encoder2Count)*PULSE_TO_UM/deltaTime;

  data.encoder1Count = tempEncoderCount1;
  data.encoder2Count = tempEncoderCount2;
  dSpeed1 = targetSpeed1 - encoderSpeed1;
  dSpeed2 = targetSpeed2 - encoderSpeed2;
  if(dSpeed1 > MAX_ERR && Motor1Power != 127) {
    Motor1Power++;
  }
  else if(dSpeed1 < -MAX_ERR && Motor1Power != -128) {
    Motor1Power--;
  }
  if(dSpeed2 > MAX_ERR && Motor2Power != 127) {
    Motor2Power++;
  }
  else if(dSpeed2 < -MAX_ERR && Motor2Power != -128) {
    Motor2Power--;
  }

  #if DEBUG
  if (Motor1Power != 0 && Motor2Power != 0) {
    Serial.print("Left wheel v=");
    Serial.print(encoderSpeed1);
    Serial.print(", Right wheel v=");
    Serial.print(encoderSpeed2);
    Serial.print(", Sending: m1=");
    Serial.print(Motor1Power);
    Serial.print(", m2=");
    Serial.println(Motor2Power);
  }
  #endif
  
  // Send powers to motors
  ST.motor(1,Motor1Power);
  ST.motor(2,Motor2Power);
  lastLoopTime = now;
}

/**
 * Callback for receiving i2c data
 */
void receiveData(int byteCount){
  
  // Buffer to hold the commands read from the motor
  int8_t readBuffer[MOTOR_COMMAND_SIZE];
  uint8_t i = 0;
  
  while(Wire.available()) {
    // Makes sure we aren't getting junk from the i2c line 
    if (byteCount == MOTOR_COMMAND_SIZE) {
      // save the data into the buffer
      readBuffer[i++] = Wire.read();
    }
    else {
      // clear our buffer
      Wire.read();
    }
  }
  
  // make sure that we read the correct number of bytes and that we have a move command
  // and update motor powers
  if (i == MOTOR_COMMAND_SIZE && readBuffer[0] == MOTOR_COMMAND) {
    targetSpeed1 = (readBuffer[1] << 24)+(readBuffer[2] << 16)+(readBuffer[3] << 8)+readBuffer[4];
    targetSpeed2 = (readBuffer[5] << 24)+(readBuffer[6] << 16)+(readBuffer[7] << 8)+readBuffer[8];
  
    // update the time we last received a message
    LastPiCommandTime = millis();
  }
  
  #if DEBUG
  if (Motor1Power != 0 && Motor2Power != 0) {
    Serial.print("Received ");
    Serial.print(byteCount);
    Serial.println(" bytes");
    Serial.print("Motor 1 Power: ");
    Serial.println(Motor1Power);
    Serial.print("Motor 2 Power: ");
    Serial.println(Motor2Power);
  }
  #endif
}


/**
 * Callback for i2c data request
 */
void sendData(){
  Wire.write((byte*)(&data),sizeof(EncoderData));
  
  #if DEBUG
  Serial.print("Encoder 1 Count: ");
  Serial.println(data.encoder1Count);
  Serial.print("Encoder 2 Count: ");
  Serial.println(data.encoder2Count);
  #endif
}
