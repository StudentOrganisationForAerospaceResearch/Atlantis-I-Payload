#include <time.h>
#include <SPI.h>
#include <Stepper.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Adafruit_BMP085.h> // Barometer Library
#include <SD.h>  // Default Arduino Library Supports it
#include <TinyGPS.h> //TinyGPS library

/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/
//TODO: figure out what these actually are
//All serial communication lines
#define DOWNLINK_SERIAL Serial5
#define GPS_SERIAL Serial2
#define IMU_SERIAL Serial1

//Parachute triggers
#define DROGUE_CHUTE_PIN 20

//Sampler pin
#define FAN_PIN 23

//Fan speeds (Do not exceed: max(70) min(40))
#define FAN_ARMED_ZERO_THRUST 40
#define FAN_FULL_THROTTLE 70

//Stepper Motor
#define STEPS_PER_REVOLUTION 200
#define STEPPER_SPEED 100
#define STEPS_PER_FILTER 25

#define STEPPER_PIN_IN1A 34
#define STEPPER_PIN_IN2A 38
#define STEPPER_PIN_IN1B 36
#define STEPPER_PIN_IN2B 39

//SD card
/*
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 */
#define SD_CS_PIN SS

// FLIGHT OPTIONS
/*****************************************************************/
#define MAIN_CHUTE_DEPLOYMENT_ALTITUDE 1000 //In meters above initial altitude
#define NUM_FILTERS 6
#define ALTITUDE_BUFFER_SIZE 10
#define ALTITUDE_TOLERANCE 50 //In meters

//Sensor Constants and defintions
#define SEA_LVL_PRESSURE 101800 //current sea level equivalent pressure for your location. used for calibration. 


// DEBUG OPTIONS
/*****************************************************************/
//TODO: add debugging options

/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/

//Output files
File dataFile;
File logFile;

//Peripheral objects
Servo samplerFan;
Stepper samplerStepper(STEPS_PER_REVOLUTION,STEPPER_PIN_0,STEPPER_PIN_1,STEPPER_PIN_2,STEPPER_PIN_3);
Adafruit_BMP085 baro;
TinyGPS gps;

//Sensor variables
float x_accel;
float y_accel;
float z_accel;

float ang_accel_x;
float ang_accel_y;
float ang_accel_z;

float mag_x;
float mag_y;
float mag_z;

float altitude;
float temperature;
float pressure;

float pitch;
float yaw;
float roll;

long longitude;
long latitude;
float altitude_gps;

//Keep track of what filter we're on
int filterNumber;

boolean mainChuteDeployed;

/*
Setup method. Initiates all variables and files
*/
void setup() {
  Serial.begin(9600);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  DOWNLINK_SERIAL.begin(9600);
  GPS_SERIAL.begin(9600);
  IMU_SERIAL.begin(57600);

  filterNumber = 0;
  mainChuteDeployed = false;

  samplerFan.attach(FAN_PIN);
  samplerFan.write(FAN_ARMED_ZERO_THRUST);


  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Initialization failed!");
    return;
  }
  
  dataFile = SD.open("Data_MASTER.txt", FILE_WRITE);
  dataFile.println("# Data file initialised at system time " + String(millis(), DEC));
  dataFile.println("Columns in order w/(units): ");

  logFile = SD.open("LOGS.txt", FILE_WRITE);
}

/*
main loop
we will use sub loops like what we planned for the c++
*/
void loop() {
	// loop to use after payload initialised
	updateData();
  float altitude_baseline = altitude;
  
	while(true)
	{
		if(altitude > ALTITUDE_TOLERANCE + altitude_baseline) //Compare against initial height
		{
			 logFile.println("Altitude change of 50 meters exceeded at " + String(millis(), DEC));
			 goto loop_launch_started;
		}
		updateData();
	}
	
loop_launch_started:
	//TODO: we need accurate acceleration curves, 15 m/s^2 is the initial guess
	while(true)
	{
		if(pow(pow(x_accel, 2) + pow(y_accel, 2) + pow(z_accel,2), 0.5) <= 15) //Take the magnitude of acceleration and wait until it is smaller than 15 m/s^2
		{
			 logFile.println("Acceleration dropped to normal levels at "+ String(millis(), DEC));
			 goto loop_high_acceleration;
		}
		updateData();
	}


loop_high_acceleration:
  //take data while waiting for max height
	float altitudeBuffer[ALTITUDE_BUFFER_SIZE];
  float averageSecondHalf;
  float averageFirstHalf;
  memset(altitudeBuffer,0,sizeof(altitudeBuffer));
  
	while(true)
	{
    //Update altitude buffer
    for(int i = 1; i < ALTITUDE_BUFFER_SIZE; i++){altitudeBuffer[i-1]=altitudeBuffer[i];}
    altitudeBuffer[ALTITUDE_BUFFER_SIZE-1] = altitude;

    //Average the first half of the buffer
    averageFirstHalf = 0.0;
    for(int i = 0; i < floor(ALTITUDE_BUFFER_SIZE/2.0); i++){averageFirstHalf+= altitudeBuffer[i];}
    averageFirstHalf = averageFirstHalf/floor(ALTITUDE_BUFFER_SIZE/2.0);

    //Average the second half of the buffer
    averageSecondHalf = 0.0;
    for(int i = floor(ALTITUDE_BUFFER_SIZE/2.0); i < ALTITUDE_BUFFER_SIZE; i++){averageSecondHalf += altitudeBuffer[i];}
    averageSecondHalf = averageSecondHalf/(ALTITUDE_BUFFER_SIZE - floor(ALTITUDE_BUFFER_SIZE/2.0));

    //Check for descent by checking if the first half of the buffer shows us being higher than the second half
    if(averageFirstHalf > averageSecondHalf){goto loop_begun_descent;}
    else{updateData();}
	}
 
loop_begun_descent:
  deployChute(DROGUE_CHUTE_PIN);
  
	//begins the spin sampler sequence
  samplerFan.write(FAN_FULL_THROTTLE);
	spinSampler();
	while(true) //turn the sampler while we fall until second parachute deployment
	{
		 updateData();

     //TODO: implement sampler decision structure
      
     if (filterNumber == 0){goto loop_final_descent;}
	}

loop_final_descent:
  // Kill fan
  samplerFan.write(FAN_ARMED_ZERO_THRUST);
	
	// After both parachutes have deployed
	while(true)
	{
		updateData();
	}
}

/*
Function to fetch and update all data sources and listeners
*/
void updateData() {
  //Updates barometer variables
  pressure = baro.readPressure();
  altitude = baro.readAltitude(SEA_LVL_PRESSURE);
  temperature = baro.readTemperature();

  //Updates gps variables
  parseGPSStream();

  //Updates IMU vaiables
  parseIMUStream();
  
  // Prepares string for logging and telemetry purposes
  String dataString = 
    "$" + 
    String(millis()) + "|" + 
    
    String(temperature) + "|" + 
    String(pressure) + "|" +
    String(altitude) + "|" + 

    String(ang_accel_x) + "|" +
    String(ang_accel_y) + "|" +
    String(ang_accel_z) + "|" +
    
    String(x_accel) + "|" +
    String(y_accel) + "|" +
    String(z_accel) + "|" +
    
    String(mag_x) + "|" + 
    String(mag_y) + "|" + 
    String(mag_z) + "|" +

    String(pitch) + "|" + 
    String(roll) + "|" + 
    String(yaw)  + "|" +
    
    String(latitude) +"|" +
    String(longitude) + "|" +
    String(altitude_gps) +
    "*";
    
  while(sizeof(dataString) < 250){
    dataString=dataString+"*";
  }
  
  dataFile.println(dataString);
  dataFile.flush();
  DOWNLINK_SERIAL.print(dataString);
}


/******************************************************************************
  Complete parachute deployment sequence while collecting data
******************************************************************************/
void deployChute(int chutePin)
{
  unsigned long timeNow = millis();
  unsigned long targetTime;

  digitalWrite(chutePin, HIGH); //fire parachute 
  logFile.println("Parachute " + String(chutePin, DEC) + " fired " + String(timeNow, DEC) + " milliseconds after program start and at an altitude of " + String(altitude, DEC));
  
  targetTime = timeNow + 4000; //wait 4 seconds to ensure chute is deployed and only one filter is contaminated at once
  while(timeNow < targetTime)
  {
    updateData();
    timeNow = millis();
  }

  digitalWrite(chutePin, LOW); //Stop sending signal to fire parchutes in case of a short
}

/******************************************************************************
  Spin sampler and keep track of which sample it's on
******************************************************************************/
void spinSampler() {
  if (filterNumber <= NUM_FILTERS){  // So it won't fire if it is activated too many times
    samplerStepper.step(STEPS_PER_FILTER);
    filterNumber++;
    logFile.println("Spun to filter " + String(filterNumber, DEC) + " at " +  String(altitude, DEC) + " meters above sea level and at approximately " + String(millis(), DEC) + " seconds after startup");
  }
}

/******************************************************************************
  Parse IMU and assign global variables.
******************************************************************************/
void parseIMUStream() {
  String temp;
  
  while (IMU_SERIAL.available()){
    temp = IMU_SERIAL.readString();
  }

  
}

/******************************************************************************
  Parse GPS and assign glabl variables.
******************************************************************************/
void parseGPSStream() { 
  while (GPS_SERIAL.available()){
    int c = GPS_SERIAL.read();
    if(gps.encode(c)){//if statement is true if a full string has been read
      if(gps.satellites() > 1){ // Is this needed?
        gps.get_position(&latitude, &longitude); //assigns by reference
        altitude_gps = gps.altitude()/100.0;
      }
    }
  }
}
