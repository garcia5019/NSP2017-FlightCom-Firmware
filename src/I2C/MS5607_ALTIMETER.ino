/*
 * Project HLM_ALTIMETER_MODULE
 * Description: HLM1 Altimeter/Temperature/Pressure I2C Module
 * Author: ASFM HLM STEM LAB - 2017 Near Space Team
 * License: MIT Open Source
 * Date: January/2018
 */

#include <MS5611.h>
#include <MS5xxx.h>

//
// ALTIMETER/TEMPERATURE/PRESSURE MODULE
//

// I2C
#include <Wire.h>

// ALTIMETER MODULE: MS5607
MS5xxx sensor(&Wire);

// temperature storage
char t[10]; //empty array where to put the numbers going to the master
float temperature = 0.0;

// pressure storage
char p[10]; //empty array where to put the numbers going to the master
float pressure = 0.0;

bool tOrP = true; // true -> temperature, false -> pressure

void setup() {
  
  Serial.begin(9600);
  
  Wire.begin(1); // Join the I2C Bus as Slave on address 1
  Wire.onRequest(requestEvent); // Function to run when master asking for data
  Wire.onReceive(receiveEvent);
  
  Serial.println("SETUP DONE");
  
}

void loop() {

  sensor.ReadProm();
  sensor.Readout();
  
  Serial.print("Temperature [0.01 C]: ");
  Serial.println(sensor.GetTemp());
  temperature = sensor.GetTemp();
  
  Serial.print("Pressure [Pa]: ");
  Serial.println(sensor.GetPres());
  pressure = sensor.GetPres();

  // store in char arrays
  // dtostrf converts the float variables to a string for I2C. (floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, empty array);
  dtostrf(temperature, 3, 2, t); 
  dtostrf(pressure, 3, 2, p); 
  
  test_crc();
  Serial.println("---");
  
  delay(5000);
}

void requestEvent() {
  if (tOrP == 0) {
      // write pressure
      Wire.write(p);
  } else if (tOrP == 1) {
      // write temperature
      Wire.write(t);
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
// this function should recieve only either a 0 or 1
void receiveEvent(int howMany) {

  // while loop should only be used if transmiting more than 1 byte of data.
  // This will happen when sat Arduino connected to more than 2 sensors.
  
  // while (1 < Wire.available()) { // loop through all but the last
    //char c = Wire.read(); // receive byte as a character
    //Serial.print(c);         // print the character
  //}
  
  tOrP = Wire.read();    // receive byte as an integer
}

void test_crc() {
  sensor.ReadProm();
  sensor.Readout(); 
  Serial.print("CRC=0x");
  Serial.print(sensor.Calc_CRC4(), HEX);
  Serial.print(" (should be 0x");
  Serial.print(sensor.Read_CRC4(), HEX);
  Serial.print(")\n");
  Serial.print("Test Code CRC=0x");
  Serial.print(sensor.CRCcodeTest(), HEX);
  Serial.println(" (should be 0xB)");
}
