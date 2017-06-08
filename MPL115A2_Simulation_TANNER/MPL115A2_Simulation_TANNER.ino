



/*Simulation of a rocket launch to demonstrate the MPL15A2 Barometer

Currently being tested on a Arduino Nano using the following I2C connections

MPL115A2      Arduino
---------------------------
VDD =========> 3V3 or 5V
GND =========> GND
SDA =========> A4 (SDA)
SCL =========> A5 (SCL)

*/

#include <Wire.h>
#include <Adafruit_MPL115A2.h> //Downloadable from the Arduino IDE library manager
Adafruit_MPL115A2 Sensor; //Named instance, can be whatever you want
unsigned long startTime;
unsigned long currentTime;
unsigned long previousPrint;
unsigned long previousSample;
unsigned long drogeDeployed;
unsigned long mainChuteDelay = 500; //Delay in milliseconds between droge and main chute deployment
unsigned long secondsElapsed = 0;

#define SAMPLE_RATE 500 //Sample rate in ms. Technically sample period.
#define PRINT_RATE 1000 //Print rate in ms for serial output

#define BUFFER_SIZE 4 //size of the buffer used to determine averages and changes in pressure.
float Pressure[BUFFER_SIZE];

boolean simulation = true; //Indicates if simulation code should run. 
boolean print = false;
boolean sample = false;
boolean launch = false;
boolean apoapsis = false;
boolean drogue = false;
boolean mainChute = false;



void setup() {
  Serial.begin(9600);
    
  Sensor.begin();
  
  //Set the buffer to known values
  for(int i = 0; i < BUFFER_SIZE; i++)
    Pressure[i] = Sensor.getPressure();
 
  Serial.print("Current pressure is ");
  Serial.print(Pressure[0], 4);
  Serial.println(" KPA");

  if(simulation){
  Serial.println("SIMULATION STARTED\n\nThe software will simulate a flight with apoapsis occuring at 10s and main chute being relased half second later."); 
  Serial.println("The current pressure reading from the sensor will be displayed\n");
  delay(2000);
  Serial.print("Launch in ");
  for(int i = 3; i > 0; i--){
    Serial.println(i);
    delay(1000);
  }
  Serial.println("Lift Off!");
  launch = true;
  }

  
  startTime = millis();
  Serial.print(startTime);
  previousPrint = millis();
  previousSample = millis();
}




void loop() {

  currentTime = millis();
  
  //Check if it is time to print
  if((currentTime - previousPrint) > PRINT_RATE){
    previousPrint = currentTime;
    print = true;
  }
  
  //Check if it is time to sample
  if((currentTime - previousSample) > SAMPLE_RATE){
    previousSample = currentTime;
    sample = true;
  }
  
  if(simulation){ //Force the apoapsis flag at 10 seconds for the simulation
    if(secondsElapsed == 10 && !apoapsis){
      apoapsis = true;
    }
  }
  
  if(print){ //Print whatever stats you want over serial
    print = false;
    Serial.print("Time: ");
    secondsElapsed = ((unsigned long)(currentTime - startTime)/1000);
    Serial.print(secondsElapsed);
    Serial.print(" seconds        Pressure: ");
    Serial.print(Pressure[0],4); //The most recent measurement
    Serial.println(" KPA");  
  }


  
  if(sample){
    sample = false;
    for(int i = 4; i > 0; i--){
      Pressure[i] = Pressure[i-1]; //shuffle everything through the buffer
    }
    Pressure[0] = Sensor.getPressure();
  }



  
  /*
   * Run through all of the pressure readings in the buffer. Check for consistancy of launch or freefall over the entire set of readings.
   */
  if(!simulation){ //Example code to show how it could potentially work in flight

    //Suggestion: Add a flag here to only run the following code if the accelerometer also indicates the rocket is accellerating to have additional confirmation
    if(!launch){
      launch = true;
      for(int i = 0; i < BUFFER_SIZE-1; i++){
        if(Pressure[i] > Pressure[i+1]) //Opposite of expected
          launch = false;
      } 
    }
    
    //Suggestion: Add a flag here to only run the following code if the accelerometer also indicates the rocket is in freefall to have additional confirmationif(
    if(launch && !apoapsis){
     apoapsis = true;
      for(int i = 0; i < BUFFER_SIZE-1; i++){
         if(Pressure[i] < Pressure[i+1])//Opposite of expected
          apoapsis = false;
      } 
    }

  }




  
    // If apoapsis has been reached and drogue has not been released, release it
  if (apoapsis && !drogue){
     drogue = true;
     Serial.println("APOAPSIS REACHED");
     Serial.println("DROGUE DEPLOYED");
     drogeDeployed = millis();
  }

  if(drogue && !mainChute && ((millis() - drogeDeployed)>= mainChuteDelay)){
    mainChute = true;
    Serial.println("MAIN CHUTE DEPLOYED");
  }
  
}
