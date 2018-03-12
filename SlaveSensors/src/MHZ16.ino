#include <NDIRZ16.h>

NDIRZ16 mySensor = NDIRZ16(&Serial3);
byte rx_byte = 0;  


void setup()
{
    Serial.begin(9600);
    Serial3.begin(9600);
    Serial.println("Wait...");
    delay(10000);
    Serial.println("START");
}

void loop() {

 if (mySensor.measure()) {
        Serial.print("CO2 Concentration is ");
        Serial.print(mySensor.ppm);
        Serial.println("ppm");
    } else {
        Serial.println("Sensor communication error.");
    }

    delay(1000);
  
}
