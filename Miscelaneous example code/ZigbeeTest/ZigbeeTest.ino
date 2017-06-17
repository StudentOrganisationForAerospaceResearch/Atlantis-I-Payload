#include <SoftwareSerial.h>
  SoftwareSerial mySerial(3, 4);
void setup() {
  

Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
while(mySerial.available()){
  Serial.println(mySerial.read());
}
}
