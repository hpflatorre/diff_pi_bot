/*
 Simple Leddar(TM) Example, using a LeddarOne.
 Language: Arduino
 
 This program displays on a LCD screen the distance
 and returned signal amplitude of the detection
 
   Shields used:
 * RS-485 Shield
 * LCD Keypad Shield
 
 This example code is in the public domain.
*/
#include <SoftwareSerial.h>
#include "LeddarFix.h"
#include <LiquidCrystal.h>


LeddarOne Leddar1(115200,1,12,1);
//Baudrate = 115200
//Modbus slave ID = 01
// NOTE: If your RS-485 shield has a Tx Enable (or DE) pin, 
// use: Leddar Leddar1(115200,1, TxEnablePinNumber, 1);

//LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
SoftwareSerial mySerial(8, 9); // RX, TX

void setup()
{
	//Serial.begin(115200); //Opens serial connection at 9600bps.
  //mySerial.begin(4800);
	// Initialize LCD
	//lcd.begin(16, 2);

	//Initialize Leddar 
	Leddar1.init();

  delay(1000);
  

}

void loop()
{
  
	Leddar1.setBaud(1);
	delay(2000);
}
