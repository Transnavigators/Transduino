#include <Sabertooth.h>

Sabertooth ST(128);

void setup() {
    SabertoothTXPinSerial.begin(9600);
    ST.setBaudRate(115200);
    SabertoothTXPinSerial.end();
	ST.setRamping(5);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
}
