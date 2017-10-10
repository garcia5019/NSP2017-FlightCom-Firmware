/*
 * Project NSP-HLM1
 * Description: Near Space Program @ ASFM
 * Author: NSP Team
 * Date: October/2017
 */

#include "TinyGPS++.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

#define GPS Serial1
#define GPSEvent serialEvent1

Timer timer(1000, second_tick);
int elapsedSeconds = 0;
int elapsedMinutes = 0;

int initialGPSAltitude = -1;


enum MissionState { 
	ground,
	climbing,
	apogee,
	descending,
	recovery
};

enum GPSState { 
	unknown,
	noFix,
	Fix  
};



TinyGPSPlus gpsParser;
MissionState missionState = ground;
GPSState gpsState = unknown;

int loopx = 50;

void setup() {
	RGB.control(true);
	GPS.begin(9600);
	Serial.begin(9600);	
	RGB.color(255, 255, 255);
	RGB.brightness(100);
	delay(1000);
	RGB.brightness(0);
	timer.start();


}

void loop() {}


//TIMED EVENTS
void second_tick() {
	timer.stopFromISR();
	elapsedSeconds++;

	if (elapsedSeconds % 2 == 0 ) {
		RGB.brightness(0);
	} else {
		RGB.brightness(100);
	}

	if (elapsedSeconds==30) { a30second_tick(); }
	if (elapsedSeconds==60) { a60second_tick(); }

	if (elapsedSeconds>=60) { elapsedSeconds = 0; }


	updateStatus();
	Serial.println("[State] Initial Altitude Set to : " + String(initialGPSAltitude) );
	timer.startFromISR();	
}


void a30second_tick() {	
	Serial.println(satString());
}

void a60second_tick() {
	elapsedMinutes++;
	if (elapsedMinutes>=5) { a5Minute_tick(); elapsedMinutes = 0; }

	Serial.println("60 TICK");
}

void a5Minute_tick() {
	Serial.println("5 minute TICK");
}



//State Updater
void updateStatus() {

	if (missionState == ground && initialGPSAltitude > 0 && (gpsParser.altitude.feet() - initialGPSAltitude) > 100) {
		missionState = climbing;
		Serial.println("[StateChange] climbing. Delta: " + String((gpsParser.altitude.feet() - initialGPSAltitude)) );
	}
	
}

//Helper Functions
String satString() {
  return String(gpsParser.location.lat(), 2) + "," + String(gpsParser.location.lng()) + "," + String(gpsParser.altitude.feet()) + "," + String(gpsParser.speed.knots()) + "," + String(gpsParser.course.deg()) + "," + String(gpsParser.satellites.value()) + "," + String(gpsParser.hdop.value());
}



// LISTENING EVENTS
void GPSEvent()
{
	if (GPS.available()) {
		while (GPS.available()) {
			char c = GPS.read();
			gpsParser.encode(c);
		}
	}

	if (gpsParser.hdop.value() < 300) {
		gpsState = Fix;		
			if (initialGPSAltitude==-1 && gpsParser.altitude.feet() > 0) {
				initialGPSAltitude = gpsParser.altitude.feet();
			}
	}	else {
		gpsState = noFix;		
	}

}