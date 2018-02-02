/*
 * Project HLM_I2C_MASTER
 * Description: HLM1 Master Module for I2C Slaves
 * Author: ASFM HLM STEM LAB - 2017 Near Space Team
 * License: MIT Open Source
 * Date: January/2018
 */

#include <Wire.h>

char t[10]={};//empty array where to put the numbers coming from the slave
char p[10]={};//empty array where to put the numbers coming from the slave

int tOrP = 0; // default pressure (0)

void setup() {
  // Start the I2C Bus as Master
  Wire.begin(); 
  Serial.begin(9600);
}

void loop() {

  // alternate between asking for pressure and temperature: 0/1
  Wire.beginTransmission(1); // transmit to slave device #1
  Wire.write(tOrP);
  Wire.endTransmission ();

  if (tOrP == 0) {
    
    // request pressure
    Wire.requestFrom(1, 5);    // request 5 bytes from slave device #1

    // gathers data coming from slave
    int i = 0; // counter for each byte as it arrives
    while (Wire.available()) { 
      p[i] = Wire.read(); // every character that arrives it put in order in the empty array "t"
      i = i + 1;
    }
    
    // better method for printing with a decimal point after 2 values would be nice
    Serial.print("Pressure [Pa]: ");
    Serial.println(String(p[0]) + String(p[1]) + "." + String(p[2]) + String(p[3]) + String(p[4]));   //shows the data in the array p
    Serial.println("--------------");

    tOrP = 1; // ask for temperature next
  
  } else if (tOrP == 1) {

    // request temperature
    Wire.requestFrom(1, 5);    // request 5 bytes from slave device #1

    // gathers data coming from slave
    int i = 0; // counter for each byte as it arrives
    while (Wire.available()) { 
      t[i] = Wire.read(); // every character that arrives it put in order in the empty array "t"
      i = i + 1;
    }
    
    // better method for printing with a decimal point after 2 values would be nice
    Serial.print("Temperature [ºC]: ");
    Serial.println(String(t[0]) + String(t[1]) + "." + String(t[2]) + String(t[3]) + String(t[4]));   //shows the data in the array t
    

    tOrP = 0; // ask for pressure next
  }

  delay(200); //give some time to relax
  
  
}
