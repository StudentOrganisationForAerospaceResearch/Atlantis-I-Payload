#include <Time.h>


#include<Wire.h> // for IMU communication
int T=75;
//int T=1000;
int time1=6.5;

void setup() {
  // put your setup code here, to run once:
  int inPin=33;
  int inPin2=34;
  int outPin=37;
  pinMode(inPin, INPUT);      // sets the digital pin 33 as input
  digitalWrite(inPin, LOW);
  pinMode(inPin2, INPUT);      // sets the digital pin 33 as input
  //pinMode(outPin, OUTPUT);      // sets the digital pin 37 as output
  digitalWrite(inPin2, LOW);
 // digitalWrite(outPin, LOW);
//  analogWriteFrequency(23, 14); // Teensy 3.5 pin 23 also changes to 14 Hz
//  analogWriteFrequency(22, 14); // Teensy 3.5 pin 22 also changes to 14 Hz
//  analogWriteFrequency(21, 14); // Teensy 3.5 pin 21 also changes to 14 Hz
  analogWrite(23, LOW);
  analogWrite(22, LOW);
  analogWrite(21, LOW);
   analogWrite(37, LOW);
}
void spiral_sixty(){
  digitalWrite(23,HIGH);
  digitalWrite(22,LOW);
  digitalWrite(21,HIGH);
  delay(25);
  digitalWrite(23,HIGH);
  digitalWrite(22,HIGH);
  digitalWrite(21,LOW);
 delay (25);
 digitalWrite(23,LOW);
  digitalWrite(22,HIGH);
  digitalWrite(21,HIGH);
 delay (25);

  return;
  }

void loop() {
  // put your main code here, to run repeatedly:
  int launch;
  int Purge;
  int inPin=33;
  int inPin2=34;
   int outPin=37;
  launch=digitalRead(inPin);   // read the input pin
  delay(100);
  Purge=digitalRead(inPin2);   // read the input pin
  delay(100);
  Serial.print("\r\n");
 Serial.print("waiting");
  delay(1000);
  Serial.print("\r\n Launch value is:");
  Serial.print( launch);
  Serial.print("\r\n");
  Serial.print("\r\n Purge value is:");
  Serial.print( Purge);
  Serial.print("\r\n");
  analogWrite(23,LOW);
  analogWrite(22,LOW);
  analogWrite(21,LOW);
  if(Purge==HIGH){
    Serial.print("\r\n");
     Serial.print("Purge sequence started");
     Serial.print("\r\n");
    analogWrite(23,255);
    
    delay(10000);
    analogWrite(23,LOW);
    analogWrite(22,255);
    delay(10000);
    analogWrite(22,LOW);
    analogWrite(21,255);
    delay(10000);
    analogWrite(21,LOW);
     while(1){
      analogWrite(23,LOW);
      analogWrite(22,LOW);
      analogWrite(21,LOW);
      Serial.print("\r\n");
      Serial.print("Purge complete");
      Serial.print("\r\n");
      delay(1000);
     
    Purge=digitalRead(inPin2);   // read the input pin
     }
  }
  
else if(launch==HIGH){
  analogWrite(37,255);
  Serial.print("launch sequence started");
  delay(11000);
  analogWrite(37,LOW);
  delay(50);
  for(int i; i<86;i++){
  spiral_sixty();
  }
//analogWrite(23,255);
//   delay(125);//Turn first valve on for 125ms 
//   analogWrite(22,255);
//   analogWrite(21,255);
//   delay(5000);//Turn all valves on for 5s 
   while(1){
   analogWrite(23,LOW);
   analogWrite(22,LOW);
   analogWrite(21,LOW);

  
   Serial.print("\r\n");
   Serial.print("launch complete");
   Serial.print("\r\n");
    analogWrite(37,LOW);
   launch=digitalRead(inPin);
   delay(1000);}
    
}
}
