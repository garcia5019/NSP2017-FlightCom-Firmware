/*
 * Project HLM_TEMPERATURE_MODULE
 * Description: HLM1 Temperature Sensor I2C Module
 * Author: ASFM HLM STEM LAB - 2018 Near Space Team
 * License: MIT Open Source
 * Date: February/2018
 */

//
// TMP36 COMBUSTIBLE GAS MODULE
//

//TMP36 Pin Variables
int sensorPin = 0; //the analog pin the TMP36's Vout (sense) pin is connected to
                        //the resolution is 10 mV / degree centigrade with a
                        //500 mV offset to allow for negative temperatures

// temperature storage
char t[10]; //empty array where to put the numbers going to the master
float temperature = 0.0;


void setup() {
  
  Serial.begin(9600);

  Wire.begin(1); // Join the I2C Bus as Slave on address 1
  Wire.onRequest(requestEvent); // Function to run when master asking for data
  
  Serial.println("SETUP DONE");
}
 
void loop() {
  //chou no te jalaba pq lo tienes que scale a la voltage de whatever sensor youre using – esto esta configured para el mega, el jueves te lo set up para el teensy
  //getting the voltage reading from the temperature sensor
  float reading = analogRead(sensorPin);  
 
  // converting that reading to voltage, for 3.3v arduino use 3.3  
  float voltage = (reading / 1024) * 3.3;
 
  // now print out the temperature
  float temperature = (voltage - 0.5) * 10; //converting from 10 mv per degree wit 500 mV offset
                                               //to degrees ((voltage - 500mV) times 100)
  
  //Serial.print("Temperature [0.01 C]: ");
  Serial.println(temperature);

  // store in char arrays
  // dtostrf converts the float variables to a string for I2C. (floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, empty array);
  dtostrf(temperature, 3, 2, t);
 
  delay(5000);
}

void requestEvent() {
  Wire.write(t);
}
