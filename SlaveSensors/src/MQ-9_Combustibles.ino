/*
 * Project HLM_CARBON-OXIDE_MODULE
 * Description: HLM1 Combustible Gas Sensor I2C Module
 * Author: ASFM HLM STEM LAB - 2018 Near Space Team
 * License: MIT Open Source
 * Date: January/2018
 */

#include <Wire.h>

//
// MQ-9 COMBUSTIBLE GAS MODULE
//


// ppm storage
char p[10]; //empty array where to put the numbers going to the master
float combustible_ppm = 0.0;

void setup() {
  Serial.begin(9600);

  Wire.begin(1); // Join the I2C Bus as Slave on address 1
  Wire.onRequest(requestEvent); // Function to run when master asking for data
  
  Serial.println("SETUP DONE");
}
 
void loop() {

  float sensor_volt;
  float RS_gas; // Get value of RS in a GAS
  float ratio; // Get ratio RS_GAS/RS_air
  int sensorValue = analogRead(A0);
  sensor_volt = (float) sensorValue/1024*5.0;
  RS_gas = (5.0-sensor_volt)/sensor_volt; // omit *RL
  
  ratio = RS_gas/0.03;  // ratio = RS/0.03 
  
  Serial.print("sensor_volt = ");
  Serial.println(sensor_volt);
  Serial.print("RS_ratio = ");
  Serial.println(RS_gas);
  Serial.print("RS / 0.03 = ");
  Serial.print(ratio);
  Serial.println(" ppm");
  
  Serial.print("\n\n");

  combustible_ppm = ratio;

  // store in char arrays
  // dtostrf converts the float variables to a string for I2C. (floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, empty array);
  dtostrf(combustible_ppm, 3, 2, p); 

  delay(5000);
}

void requestEvent() {
  // write combustible ppm
  Wire.write(p);
}

