#include <time>
#include <iostream>
#include <ofstream>
#include <omp.h>
#include <Wire.h>
#include <math.h>


//Important vars
int mainChuteDeploymentAltitude;
int downlinkPort;
int gpsPort;
int magnetometerPort;
int samplerFilter;


//Output file
ofstream launchFile;


//Time variables **remember time is only time after program starts**
unsigned long now; //only used to get current program run time


//Sensor variables
float lin_accel_x;
float lin_accel_y;
float lin_accel_z;

float x_accel;
float y_accel;
float z_accel;

float altitude;
float temperature;
float pressure;

float orientation;

flost coords; //modify this for output of gps


/*
Setup method. Initiates all variables and files
*/
void setup() {
  mainChuteDeploymentAltitude = 450;  // in meters above the ground
  downlinkPort; //TODO: get these port numbers
  gpsPort;
  magnetometerPort;
  samplerFilter = 0;
  
  baroFile.open("Barometer.txt", std::ofstream::out);
  accelFile.open("Accelerometer.txt", std::ofstream::out);
  gpsFile.open("GPS.txt", std::ofstream::out);
  magnetometerFile.open("Magnetometer.txt", std::ofstream::out);
  samplerFile.open("Sampler.txt", std::ofstream::out);
  
  baroFile << "Time" << " " << "Altitude" << endl;
  accelFile << "Time" << "	" << "x_accel" << " " << "y_accel" << " " << "z_accel" << endl;
  gpsFile << "Time" << " " << "Coords" << endl;
  magnetometerFile << "Time" << " " << "Orientation" << endl;
  
  CPU_PRESCALE(CPU_120MHz); // sets cpu scale
}

/*
main loop
we will use sub loops like what we planned for the c++
*/
void loop() {
	// loop to use after payload initialised
	// Record the intial height
	while(true)
	{
		if() //Compare against initial height
		{
			 //TODO: send via downlink that 10g reached
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
		if(pow(pow(x_accel, 2) + pow(y_accel, 2) + pow(z_accel), 0.5) >= 2) //Take the magnitude of acceleration and watch that it has risen to above 2 g.
		{
			 //TODO: send via downlink that 10g reached
			 ba loop_high_acceleration;
		}
		else
		{
			updatedata();
		}
	}
	
loop_high_acceleration:
  //TODO: wait for rocket to slow down to subsonic speeds (maybe change this to wait for max accel and then wait for something on the way down the accel curve)
	now = millis();
	targetTime = now + timeToWaitBeforeSubsonic;
	while(now < targetTime)
	{
		updateData();
		now = millis();
	}
	
	
  //take data while waiting for max height
	max_alt = altitude;
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
			ba loop_waiting_for_high_altitude;
		}
	}
loop_waiting_for_high_altitude:
  //the first parachute deployment then start the sampler
	now = millis();
	digitalWrite(PIN_D6, HIGH); //fire parachute ****insert correct pin
	cout << "Drogue parachute fired" << endl;
	targetTime = now + 4000; //wait 4 seconds
	while(now<targetTime)
	{
		updateData();
		now = millis();
	}
	
	//begins the spin sampler sequence
	spinSampler()
	
	digitalWrite(PIN_D6, LOW);
	
	/*sampler should finish taking data somewhere in this loop*/
	//
	while(altitude > mainChuteDeploymentAltitude op) //turn the sampler while we fall until second parachute deployment
	{
			//TODO: implement sampler decision structure
			updateData();
	}
	
	/*fire the second parachute*/
	now = millis();//make sure that sampler is back to first sample
	digitalWrite(PIN_D6, HIGH); //fire parachute ****insert correct pin
	cout << "Main parachute fired" << endl;
	targetTime = now + timeToWaitBeforeSubsonic;
	while(now<targetTime)
	{
		updateDate();
		now = millis();
	}
	
	// After both parachutes have deployed
	while(true)
	{
		updateData();
		//TODO: send gps coordinates via downlink
	}
}


/*
Main support function to update all the data
updatedata will call writeData and sendAllData at the end of it
and the current time
*/
private void updateData() {
  
  
  writeData();
  sendAllData();
}

/*
Second support function to write all data to SD card.
*/


	string baro_str;
	string_acc_str;
	str_gps_str;
	str mag_str;
private void writeData(unsigned long ctime) {
	now = milis();
	baro_str=now+" "+altitude+"\n";
	acc_str=now+' '+ x_accel + " " + y_accel + " " + z_accel + '\n';
	gps_str=  now +' ' + coords + '\n';
	mag_str= now + ' ' +coords+ '\n';
	baroFile << baro_str;
	accelFile << acc_str;
	gpsFile << gps_str;
	magnetometerFile << mag_str;
}

/*
Third support function to send all data to computer via downlink
*/
private void sendAllData() {
  //TODO: send all vars through downlink
}

/*
Method to spin the sampler and keep track of what filter it's on.
*/
private void spinSampler(int spinToStart) {
	if (spinToStart == 0) {
	  while (samplerFilter = 0) {
	    //TODO: spin sampler
	    
	    samplerFilter++;
	  }
	  samplerFile << "Spun to beginning at " << altitude << " meters above sea level and at approximately " << now << " seconds after startup" << endl;
	}
   
  //TODO: spin sampler
  samplerFilter++;
  
  samplerFile << "Spun to filter " samplerFilter << " at " << altitude << " meters above sea level and at approximately " << now << " seconds after startup" << endl;
  //TODO: Send via downlink
}
