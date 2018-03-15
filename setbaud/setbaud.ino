/*!
   \file setbaud.ino
   \brief Configures the Sabertooth motor controller for the Transnavigators' Voice Controlled Wheelchair

  Interfaces over UART with a <a href="https://www.dimensionengineering.com/products/sabertooth2x60">Sabertooth 2x60</a> on pin 1.

  \mainpage SetBaud

*/
#include <Sabertooth.h>

//! The Sabertooth motor controller object from the <a href="https://www.dimensionengineering.com/software/SabertoothArduinoLibrary/html/index.html">Sabertooth library</a>
Sabertooth ST(128);

//! Setup routine, sets the baud rate to 115200 and sets ramping to 5
void setup() {
    SabertoothTXPinSerial.begin(9600);
    ST.setBaudRate(115200);
    ST.setRamping(5);
    SabertoothTXPinSerial.end();
    SabertoothTXPinSerial.begin(115200);
    ST.setRamping(5);
}

//! Main loop, does nothing
void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
}
