void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("Hello World!");
}

void loop() {
    char buff[100];
    if(Serial.available()) {
      size_t numChars = Serial.readBytesUntil('!',buff,100);
      buff[numChars] = '\0';
      Serial.write("You said: ");
      Serial.write(buff);
      Serial.write('\n');
      Serial.flush();
    //}
  }
}
