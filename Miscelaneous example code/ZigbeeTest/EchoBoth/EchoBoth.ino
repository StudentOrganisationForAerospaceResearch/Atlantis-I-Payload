/* UART Example, any character received on either the real
   serial port, or USB serial (or emulated serial to the
   Arduino Serial Monitor when using non-serial USB types)
   is printed as a message to both ports.

   This example code is in the public domain.
*/

// set this to the hardware serial port you wish to use
#include <SoftwareSerial.h>
#define DOWNLINK_SERIAL Serial


void setup() {
   DOWNLINK_SERIAL.begin(9600);
  // mySerial.begin(9600);
   Serial4.begin(9600);
}

void loop() {
  bool stringNotRead=true;
  String input;
  
 // while (stringNotRead && DOWNLINK_SERIAL.available()){
//    if(DOWNLINK_SERIAL.read() == '$'){
//      input = DOWNLINK_SERIAL.readStringUntil('*');
//      input = "$" + input + "*";
//      stringNotRead=false;
//    }
//  }
        delay(200);
  Serial4.write("testing");
  }


