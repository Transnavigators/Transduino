#include <Wire.h>
#include <VL6180X.h>
#include <FastCRC.h>

//The VL6180Xs will have addresses 0x30, 0x31...
uint8_t NEWVL6180X_ADDRESS = 0x30;

//The sensors, their enable pins, and timeout in ms
const int NUM_SENSORS = 2;
VL6180X sensor[2];
const int enable[] = {5, 6};
const int sensorTimeout = 10;

//The period to check the sensors in us
int period = 10000;

//The next time in us to read the sensors
unsigned long nextSensorTime = 0;

struct RangePacket {
  uint8_t Address;
  uint8_t PacketID;
  uint16_t dist1;
  uint16_t dist2;
  uint16_t CRC;
};

/*void setup() {
  initIRSensors();
}*/

void initIRSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    //Set enable pins to outputs and set them all low
    pinMode(enable[i], OUTPUT);
    digitalWrite(enable[i], LOW);
  }

  //Start wire library for VL6180X
  Wire.begin();

  //Start initializing the sensors one by one
  for (int i = 0; i < NUM_SENSORS; i++) {
    //Turn on the next sensor
    digitalWrite(enable[i], HIGH);

    //Wait 0.1s for power off to happen before sending commands
    delay(100);

    //Initialize the sensor and change its address to something unique
    sensor[i].init();
    sensor[i].configureDefault();
    delay(500);
    sensor[i].setAddress(NEWVL6180X_ADDRESS++);
    sensor[i].setTimeout(sensorTimeout);
  }

  nextSensorTime = micros() + period;
}

//Asynchronously measure range and construct packet
void sendRange() {
  RangePacket rp;
  rp.Address = 0xEE;
  rp.PacketID = 0x03;
  rp.dist1 = sensor[0].readRangeContinuousMillimeters();
  rp.dist2 = sensor[1].readRangeContinuousMillimeters();

  //Calculate CRC for packet
  rp.CRC = CRC16.ccitt((const uint8_t*)&rp,48);

  //Check if the measurement timed out
  for (int i = 0; i < 2; i++) {
    if (sensor[i].timeoutOccurred()) {
      period += 1000;
      Serial.print("Sensor " + String(i) + " timed out. Setting period to " + String(period) + " us.");
      nextSensorTime = micros() + period;
      return;
    }
  }

  //Send the packet
  const byte* p = (const byte*) &rp;
  for (int i = 0; i < sizeof rp; i++) {
    Serial.write(*p++);
  }
  Serial.flush();

  //The next time to read the sensor
  nextSensorTime = micros() + period;
}

/*void loop() {
  unsigned long currTime = micros();
  if (currTime > nextSensorTime) {
    sendRange();
  }
}*/
