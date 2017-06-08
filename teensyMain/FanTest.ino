#include <time.h>
#include <SD.h>
#include <Wire.h>
#include <Stepper.h>

#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Stepper.h>
/******************************************************************************
 Flight Configuration Parameters
******************************************************************************/
const int SAMPLER_FAN_ENABLE_PIN = 23;// for sampler intake fan throttle
const int FAN_ARMED_ZERO_THRUST = 40;
const int FAN_FULL_THROTTLE = 70;     // do NOT exceed these limits
Servo samplerFan;
bool sampler_fan_running = false;

void setup() {
  // put your setup code here, to run once:
  samplerFan.attach(SAMPLER_FAN_ENABLE_PIN);
  samplerFan.write(FAN_ARMED_ZERO_THRUST); // set the fan to zero speed but 
                                           // arm it in the initialization
}



void loop() {
  // put your main code here, to run repeatedly:
  delay(5000);
 samplerFan.write(FAN_FULL_THROTTLE);
 delay(5000);
 samplerFan.write(FAN_ARMED_ZERO_THRUST);
 delay(10000000);
}
