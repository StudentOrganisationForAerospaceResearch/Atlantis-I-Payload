/*Quick test of the Trimble GPS on the payload board.
 * The Trimble has been set up to deliver a GGA string once a second
 * over Serial2 (pin 9)
 */

#include <TinyGPS.h> //TinyGPS library
//Download here: http://arduiniana.org/libraries/tinygps/
TinyGPS gps;

void setup() {
  pinMode(9, INPUT); //Not sure if necessary
  Serial.begin(9600); //USB serial simply for demonstration purposes
  Serial2.begin(4800); //start serial 2 at 4800 baud
}

void loop() {
 while(Serial2.available()){ //Let this run in the main loop as fast as you can
    int c = Serial2.read();
    if(gps.encode(c)){ //if statement is true if a full string has been read
    long lat, lon;
    unsigned long altitude; 
    int altitude_m;
    gps.get_position(&lat, &lon);
    Serial.print("lat: ");
    //Coordinates are printed as a single digit DDHHHMMM which can be processed afterwards
    Serial.print(lat);
    Serial.print("  lon:  ");
    Serial.println(lon);
    Serial.print("satellites    ");
    Serial.println(gps.satellites());

    altitude = gps.altitude();
    altitude_m = altitude/100; //altitude is in cm so need to divide by 100

    Serial.print("altitude  ");
    Serial.print(altitude_m);
    Serial.println(" m");
    }
  }
  
  
}
