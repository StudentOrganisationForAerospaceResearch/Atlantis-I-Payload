#include <time.h>
#include <SPI.h>
#include <Stepper.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Adafruit_BMP085.h>
#include <SdFat.h>

/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/
//TODO: figure out what these actually are
//All serial communication lines
#define DOWNLINK_SERIAL Serial1
#define GPS_SERIAL Serial2
#define IMU_SERIAL Serial3

//Parachute triggers
#define MAIN_CHUTE_PIN 22
#define DROGUE_CHUTE_PIN 21

//Sampler pin
#define FAN_PIN 23

//Fan speeds (Do not exceed: max(70) min(40))
#define FAN_ARMED_ZERO_THRUST 40
#define FAN_FULL_THROTTLE 70

//Stepper Motor
#define STEPS_PER_REVOLUTION 200
#define STEPPER_SPEED 100
#define STEPS_PER_FILTER 25

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
#define NUM_FILTERS 7
#define ALTITUDE_BUFFER_SIZE 10

// DEBUG OPTIONS
/*****************************************************************/
//TODO: add debugging options

/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/

//Output files
SdFat SD;
File dataFile;
File logFile;

//Time variables **time recorded is time after program starts**
unsigned long timeNow;
unsigned long targetTime;

Servo samplerFan;
Stepper samplerStepper; 

//Sensor variables
float lin_accel_x;
float lin_accel_y;
float lin_accel_z;

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

//TODO: probably need something to record N/S for coords
float longitude;
float latitude;

//Keep track of what filter we're on
int samplerFilter;

boolean mainChuteDeployed;

/*
Setup method. Initiates all variables and files
*/
void setup() {
  Serial.begin(9600);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  DOWLINK_SERIAL.begin(9600);
  GPS_SERIAL.begin(9600);
  IMU_SERIAL.begin(57600);

  samplerFilter = 0;
  mainChuteDeployed = false;

  samplerFan.attach(FAN_PIN);
  samplerFan.write(FAN_ARMED_ZERO_THRUST);

  samplerStepper = myStepper(stepsPerRevolution, 26, 27, 28, 25);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
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
	//TODO: Record the intial height
	while(true)
	{
		if() //Compare against initial height
		{
			 logFile.println("Altitude change of 50 meters exceeded at " + String(millis(), DEC));
			 ba loop_high_acceleration;
		}
		else
		{
			updatedata();
		}
	}
	
loop_launch_started:
	// we need accurate acceleration curves, 2 g is the initial guess
	while(true)
	{
		if(pow(pow(x_accel, 2) + pow(y_accel, 2) + pow(z_accel), 0.5) <= 15) //Take the magnitude of acceleration and wait until it is smaller than 15 m/s^2
		{
			 logFile.println("Acceleration dropped to normal levels at "+ String(millis(), DEC));
			 ba loop_high_acceleration;
		}
		updatedata();
	}
	
loop_high_acceleration:
  //take data while waiting for max height
	float altitudeBuffer[ALTITUDE_BUFFER_SIZE] = {altitude};
  
	while(true)
	{
		if(max_alt - 20 > altitude)
		{
			max_alt = altitude;
			updateData();
		}
		else
		{
		  baroFile << "Max altitude: " << altitude << endl;
		  //TODO: send max alt via downlink
			ba loop_begun_descent;
		}
	}
 
loop_begun_descent:
  deployChute(DROGUE_CHUTE_PIN);
  
	//begins the spin sampler sequence
  samplerFan.write(FAN_FULL_THROTTLE);
	spinSampler()
	while(true) //turn the sampler while we fall until second parachute deployment
	{
		  //TODO: implement sampler decision structure
  		updateData();
      if (altitude <= MAIN_CHUTE_DEPLOYMENT_ALTITUDE) {deployChute(MAIN_CHUTE_PIN);}
      if (samplerFilter == 0){ba loop_final_descent;}
	}
  samplerFan.write(FAN_ARMED_ZERO_THRUST);

loop_final_descent:
	// After both parachutes have deployed
	while(true)
	{
		updateData();
	}
}

//Deploy parachute
private void deployChute(int chutePin)
{
  timeNow = millis()
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

/*
Function to fetch and update all data sources and listeners
*/
std::string dataString

private void updateData() {
  //TODO: Get and interpret data
  
  timeNow = milis();
  baro_str= timeNow + " " + altitude + "\n";
  acc_str= timeNow + ' '+ x_accel + " " + y_accel + " " + z_accel + '\n';
  gps_str= timeNow +' ' + coords + '\n';
  mag_str= timeNow + ' ' +coords+ '\n';
  
  dataFile.println(data);
  DOWLINK_SERIAL.print(dataString);
}

/*
Method to spin the sampler and keep track of what filter it's on.
*/
// Pins are 26,27,28,25
private void spinSampler() {
  samplerStepper.step(STEPS_PER_FILTER);
  samplerFilter++;
  
  logFile.println("Spun to filter " + String(samplerFilter, DEC) + " at " +  String(altitude, DEC) + " meters above sea level and at approximately " + String(millis(), DEC) + " seconds after startup");
}
