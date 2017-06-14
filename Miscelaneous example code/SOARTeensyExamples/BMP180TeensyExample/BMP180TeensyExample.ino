#include <Adafruit_BMP085.h>

#include <Wire.h>

Adafruit_BMP085 Sensor; //Named instance, can be whatever you want
unsigned long startTime;
unsigned long currentTime;
unsigned long previousPrint;
unsigned long previousSample;
float Pressure;
float Altitude;

#define SAMPLE_RATE 1000 //Sample rate in ms. Technically sample period.

#define SEA_LVL_PRESSURE 100600 //current sea level equivalent pressure for your location. used for calibration. 
//Calgary data from here: https://calgary.weatherstats.ca/charts/pressure_sea-hourly.html

void setup() {
  Serial.begin(9600);
  Sensor.begin();
  startTime = millis();
}


void loop() {
  if((millis() - startTime) > SAMPLE_RATE){
    startTime = millis();
    Pressure = Sensor.readPressure();
    Altitude = Sensor.readAltitude(SEA_LVL_PRESSURE);
   
    Serial.print("Pressure is ");
    Serial.print(Pressure, 3);
    Serial.println(" KPA");
    Serial.print("Altitude is ");
    Serial.print(Altitude, 2);
    Serial.println(" m");
  }
}
