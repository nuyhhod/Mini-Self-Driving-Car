/*
This program writes the values in the address of EEPROM. 
EEPROM will read values from accelerometer sensor and store them in multiple addresses.
It will print out maximum acceleration in the x and y direction of the car. 
I will send a text to the arduino via bluetooth and then the arduino will save the maximum accelerations into the EEPROM then read it via this file. 
*/

//!!! Collaboration with Matt and Roderick and helped them understand how to use EEPROM and make a second file to read the EEPROM
#include <EEPROM.h> //EEPROM library to import its functions

int addr1 = 5;   // Address where data was stored 1st click for Ultrasonic sensor
int addr2 = 6;   // Address where data was stored 2nd click for Ultrasonic sensor

void setup() {
  Serial.begin(115200); //Serial begin to print to serial monitor
}

void loop() {
  // Read the stored data from EEPROM
  int storedData1 = EEPROM.read(addr1); //reads data stored in address 5
  int storedData2 = EEPROM.read(addr2); //reads data stored in address 6


  // Print the stored data to the Serial Monitor
  //prints the distance between sensor and object
  Serial.println("Maximum Acceleration of X-axis ");
  Serial.print(storedData1); //prints maximum acceleration of x-axis from EEPROM address 5
  Serial.println("  m/s^2"); //prints m/s^2

  Serial.println("Maximum Acceleration of Y-axis ");
  Serial.print(storedData2); //prints maximum acceleration of x-axis from EEPROM address 6
  Serial.println("  m/s^2"); //prints m/s^2
  delay(100);  // Change Delay
}