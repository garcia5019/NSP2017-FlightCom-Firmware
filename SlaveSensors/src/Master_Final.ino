/*
 * Project HLM_I2C_MASTER
 * Description: HLM1 Master Module for I2C Slaves
 * Author: ASFM HLM STEM LAB - 2017 Near Space Team
 * License: MIT Open Source
 * Date: January/2018
 */

#include <Wire.h>

// TMP36_Temperature STORAGE
char tempTMP36[10]={}; // empty array for temperature

// MS5607_Altimeter STORAGE
char tempMS5607[10]={}; // empty array for temperature
char presMS5607[10]={}; // empty array for pressure

// DHT22_Humidity STORAGE
char humDHT22[10]={}; // empty array for humidity
char tempDHT22[10]={}; // empty array for temperature

int valueRequest = 0; // 0: TMP36-TEMPERATURE, 
                      // 1: MS5607-TEMPERATURE,
                      // 2: MS5607-PRESSURE,
                      // 3: DHT22-TEMPERATURE,
                      // 4: DHT22-HUMIDITY

int maxValue = 4;

void setup() {
  // Start the I2C Bus as Master
  Wire.begin(); 
  Serial.begin(9600);
}

void loop() {

  // alternate between sensors
  Wire.beginTransmission(1); // transmit to Teensy on LINE 1
  Wire.write(valueRequest);
  Wire.endTransmission ();

  if (valueRequest == 0) { // TMP36-TEMPERATURE
    
    // request TMP36-TEMPERATURE
    requestTeensy(valueRequest, tempTMP36, "Temperature [ºC]");
  
  } else if (valueRequest == 1) { // MS5607-TEMPERATURE

    // request MS5607-TEMPERATURE
    requestTeensy(valueRequest, tempMS5607, "Temperature [ºC]");
    
  } else if (valueRequest == 2) { // MS5607-PRESSURE

    // request MS5607-PRESSURE
    requestTeensy(valueRequest, presMS5607, "Pressure [Pa]");
    
  } else if (valueRequest == 3) { // DHT22-TEMPERATURE
    
    // request DHT22-TEMPERATURE
    requestTeensy(valueRequest, tempDHT22, "Temperature [ºC]");
    
  } else if (valueRequest == 4) { // DHT22-HUMIDITY

    // request DHT22-HUMIDITY
    requestTeensy(valueRequest, humDHT22, "Relative Humidity [%]");
    
  }

  delay(200); //give some time to relax
  
  
}

void requestTeensy(int value, char store[10], String unit) {

   if (valueRequest <= maxValue) {
      Wire.requestFrom(1, 5);    // request 5 bytes from Teensy (standard)

      // gather data coming from Teensy
      int i = 0; // counter for each byte as it arrives
      while (Wire.available()) { 
        store[i] = Wire.read(); // every character that arrives is written to empty array 'store'
        i = i + 1;
      }
    
      // better method for printing with a decimal point after 2 values would be nice
      Serial.print(unit + ": ");
      Serial.println(String(store[0]) 
                      + String(store[1]) + "." 
                      + String(store[2]) 
                      + String(store[3]) 
                      + String(store[4]));   //shows the data in the array 'store'
      Serial.println("--------------");

      valueRequest += 1; // ask for next value next time
      
   } else {
    
      valueRequest = 0; // if greater than maxValue, reset
   }
  
   
}

