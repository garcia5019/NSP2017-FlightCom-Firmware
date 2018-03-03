/*
 * Project HLM_TEENSY_SLAVE_MODULE
 * Description: HLM1 TEMPERATURE/PRESSURE/HUMIDITY Teensy I2C Slave Module
 * Author: ASFM HLM STEM LAB - 2018 Near Space Team
 * License: MIT Open Source
 * Date: March/2018
 */

#include <Wire.h>

#include <MS5611.h>
#include <MS5xxx.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

//
// FINAL COMBO: TEMPERATURE/PRESSURE/HUMIDITY MODULE
//

// I2C
#include <Wire.h>

// ALTIMETER MODULE: MS5607
MS5xxx MS5607(&Wire);

// TEMPERATURE MODULE: TMP36
int sensorPin = 0; //the analog pin the TMP36's Vout (sense) pin is connected to
                   //the resolution is 10 mV / degree centigrade with a
                   //500 mV offset to allow for negative temperatures

// HUMIDITY MODULE: DHT22
#define DHTPIN 2
#define DHTTYPE DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

// TMP36_Temperature STORAGE
char tempTMP36[10]={}; // empty array for temperature
float decTempTMP36 = 0.0;

// MS5607_Altimeter STORAGE
char tempMS5607[10]={}; // empty array for temperature
char presMS5607[10]={}; // empty array for pressure
float decTempMS5607 = 0.0;
float decPresMS5607 = 0.0;

// DHT22_Humidity STORAGE
char humDHT22[10]={}; // empty array for humidity
char tempDHT22[10]={}; // empty array for temperature
float decHumDHT22 = 0.0;
float decTempDHT22 = 0.0;

int valueRequest = 0; // 0: TMP36-TEMPERATURE, 
                      // 1: MS5607-TEMPERATURE,
                      // 2: MS5607-PRESSURE,
                      // 3: DHT22-TEMPERATURE,
                      // 4: DHT22-HUMIDITY

int maxValue = 4;

void setup() {
  
  Serial.begin(9600);
  
  Wire.begin(1); // Join the I2C Bus as Slave on address 1
  Wire.onRequest(requestEvent); // Function to run when master asking for data
  Wire.onReceive(receiveEvent); // Function to run when master sending data

  setupDHT22();
  
  Serial.println("SETUP DONE");
  
}

void loop() {
  
  decTempTMP36 = readTMP36("t");

  decTempMS5607 = readMS5607("t");
  decPresMS5607 = readMS5607("p");

  decTempDHT22 = readDHT22("t");
  decHumDHT22 = readDHT22("h");

  // dtostrf store in char arrays
  // dtostrf converts the float variables to a string for I2C. (floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, empty array);
  dtostrf(decTempTMP36, 3, 2, tempTMP36);
  dtostrf(decTempMS5607, 3, 2, tempMS5607);
  dtostrf(decPresMS5607, 3, 2, presMS5607);
  dtostrf(decTempDHT22, 3, 2, tempDHT22);
  dtostrf(decHumDHT22, 3, 2, humDHT22);
  
  delay(5000);
}

void requestEvent() {
  
  if (valueRequest == 0) { // TMP36-TEMPERATURE
    
    // write TMP36-TEMPERATURE
    Wire.write(tempTMP36);
  
  } else if (valueRequest == 1) { // MS5607-TEMPERATURE

    // write MS5607-TEMPERATURE
    Wire.write(tempMS5607);
    
  } else if (valueRequest == 2) { // MS5607-PRESSURE

    // write MS5607-PRESSURE
    Wire.write(presMS5607);
    
  } else if (valueRequest == 3) { // DHT22-TEMPERATURE
    
    // write DHT22-TEMPERATURE
    Wire.write(tempDHT22);
    
  } else if (valueRequest == 4) { // DHT22-HUMIDITY

    // write DHT22-HUMIDITY
    Wire.write(humDHT22);
    
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  
  valueRequest = Wire.read();    // receive byte as an integer
  Serial.println(valueRequest);  // print the integer
  
}

float readTMP36(String value) {

  //getting the voltage reading from the temperature sensor
  int reading = analogRead(sensorPin);  
 
  // converting that reading to voltage, for 3.3v arduino use 3.3
  float voltage = reading * 5.0;
  voltage /= 1024.0; 
 
  // print out the voltage
  Serial.print(voltage); Serial.println(" volts");
 
  // now print out the temperature
  float temperature = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
                                               //to degrees ((voltage - 500mV) times 100)
  return temperature;
}

float readMS5607(String value) {
  
  MS5607.ReadProm();
  MS5607.Readout();
  test_crc();

  if (value == "t") {

    Serial.print("Temperature [0.01 C]: ");
    Serial.println(MS5607.GetTemp());
    float temperature = MS5607.GetTemp();

    return temperature;
    
  } else if (value == "p") {
      
    Serial.print("Pressure [Pa]: ");
    Serial.println(MS5607.GetPres());
    float pressure = MS5607.GetPres();

    return pressure;
  }
  
}

float readDHT22(String value) {
    sensors_event_t event;
    if (value == "t") { // read temperature
      
      // Get temperature event and print its value.
      sensors_event_t event;  
      dht.temperature().getEvent(&event);
      if (isnan(event.temperature)) {
        Serial.println("Error reading temperature!");
      } else {
        Serial.print("Temperature: ");
        Serial.print(event.temperature);
        Serial.println(" *C");

        return event.temperature;
      }
      
    } else if (value == "h") { // read humidity
      
      // Get humidity event and print its value.
      dht.humidity().getEvent(&event);
      if (isnan(event.relative_humidity)) {
        Serial.println("Error reading humidity!");
      } else {
        Serial.print("Humidity: ");
        Serial.print(event.relative_humidity);
        Serial.println("%");

        return event.relative_humidity;
      }
    }
}

void setupDHT22() {
  // Initialize device.
  dht.begin();
  Serial.println("DHTxx Unified Sensor Example");
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}

void test_crc() {
  MS5607.ReadProm();
  MS5607.Readout(); 
  Serial.print("CRC=0x");
  Serial.print(MS5607.Calc_CRC4(), HEX);
  Serial.print(" (should be 0x");
  Serial.print(MS5607.Read_CRC4(), HEX);
  Serial.print(")\n");
  Serial.print("Test Code CRC=0x");
  Serial.print(MS5607.CRCcodeTest(), HEX);
  Serial.println(" (should be 0xB)");
}
