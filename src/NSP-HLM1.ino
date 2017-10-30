/*
 * Project NSP-HLM1
 * Description: Near Space Program @ ASFM
 * Author: NSP Team
 * Date: October/2017
 */

#include "TinyGPS++.h"

//MISSION SETTINGS
bool cellModemEnabled = false;
float altitudeGainClimbTrigger = 20;
float altitudePerMinuteGainClimbTrigger = 5;
float altitudeLossPerMinuteForDescentDetection = 20;
float iterationsInLowDescentToTriggerRecovery = 10;
float minimumAltitudeToTriggerRecovery = 2300;


//SIM SETTINGS
bool simulationMode = false;
bool debugMode = true;
bool gpsDebugDump = false;
float simulatedApogeeAltitude = 1000;
//


SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

#define GPS Serial1
#define COMPUTER Serial
#define GPSEvent serialEvent1
#define computerEvent serialEvent


Timer timer(1000, second_tick);
int elapsedSeconds = 0;
int elapsedMinutes = 0;

int recoveryDetectionIterations = 0;

float initialGPSAltitude = -1.0;
float lastPositiveGPSAltitude = 0.0;
float lastGPSAltitude = -1.0;
float altitudePerMinute = 0.0;
float altitudeOfApogee = -1.0;
float simulatedAltitude = 0.0; //For simulation only.

//States
bool simulatedClimbEnded = false;

FuelGauge fuel;

float batteryLevel = -1.0;

String computerSerialData;

enum MissionStage { 
	ground,
	climb,
	apogee,
	descent,
	recovery
};

enum GPSState { 
	unknown,
	noFix,
	Fix  
};


TinyGPSPlus gpsParser;
MissionStage missionStage = ground;
GPSState gpsState = unknown;

int loopx = 50;

//CLOUD FUNCTIONS
bool success = Particle.function("c", computerRequest);


void setup() {
	RGB.control(true);
	GPS.begin(57600);
	Serial.begin(57600);
	setupBatteryCharger();	
	RGB.color(0, 0, 255);
	RGB.brightness(100);	
	setCellModem(cellModemEnabled);

	delay(2000);
	RGB.color(255, 255, 255);
	RGB.brightness(0);
	timer.start();
	batteryLevel = fuel.getSoC();
	Serial.println("[Stage] Ground ");


}

void loop() {}


//TIMED EVENTS
void second_tick() {
	timer.stopFromISR();
	elapsedSeconds++;


	if (missionStage == ground && Cellular.ready() == false) { RGB.color(255,165,0); } //Orange
	if (missionStage == ground && Cellular.connecting() == true) { RGB.color(50,50,255); } //Orange
	if (missionStage == ground && Cellular.ready() == true) { RGB.color(100,165,100); } //Orange
	if (missionStage == climb) { RGB.color(0,0,255); } //Blue
	if (missionStage == apogee) { RGB.color(0,255,0); } //Green
	if (missionStage == descent) { RGB.color(255,215,0); } //Yellow
	if (missionStage == recovery) { RGB.color(255,0,255); } //Magenta

	if (elapsedSeconds % 2 == 0 ) {
		RGB.brightness(0);
	} else {
		if (debugMode == true) {
			RGB.brightness(100);
		}
	}

	if (elapsedSeconds==30) { a30second_tick(); }
	if (elapsedSeconds==60) { a60second_tick(); }

	if (elapsedSeconds>=60) { elapsedSeconds = 0; }

	//Second Updates////////////	
	updateStage();
	/////////////////
	
	if (debugMode == true && gpsDebugDump == false) {
		Serial.println(satString());
	}


	timer.startFromISR();		
}


void a30second_tick() {	
	// Serial.println(satString());	
}

void a60second_tick() {
	elapsedMinutes++;
	if (elapsedMinutes>=5) { a5Minute_tick(); elapsedMinutes = 0; }

	Serial.println("60 TICK");
	batteryLevel = fuel.getSoC();
}

void a5Minute_tick() {
	Serial.println("5 minute TICK");
}



//State Updater
void updateStage() {

	float gpsAltitude = gpsParser.altitude.feet() + simulatedAltitude;	
	float altitudeGain = (gpsAltitude - initialGPSAltitude);
	float altitudeTrend = gpsAltitude - lastGPSAltitude;	
	altitudePerMinute = altitudeTrend * 60;
	
	if ((gpsAltitude < lastGPSAltitude) && (gpsAltitude > altitudeOfApogee) && lastGPSAltitude != -1)  {
		Serial.println("[Event] Apogee Reached at " + String(gpsAltitude));
		altitudeOfApogee = gpsAltitude;
	}

	lastGPSAltitude = gpsAltitude;

	
	//SIMULATOR////////////
	if (simulationMode) {
		if (simulatedClimbEnded == false && (missionStage == ground || missionStage == climb)) {
			simulatedAltitude+=50;
			if (altitudeGain >= simulatedApogeeAltitude) {
				simulatedClimbEnded = true;	
			}
		} else if (simulatedClimbEnded == true) {
			simulatedAltitude-=50;
			if (gpsAltitude <= initialGPSAltitude) {
				simulatedClimbEnded = false;
			}
		} 
	} 
	//SIMULATOR////////////


	//State Machine
	if ((missionStage == ground && initialGPSAltitude > 0) && (altitudeGain > altitudeGainClimbTrigger) && (altitudePerMinute > altitudePerMinuteGainClimbTrigger)) {
		missionStage = climb;
		lastPositiveGPSAltitude = gpsAltitude;
		Serial.println("[Stage] Climb Detected at: " + String(gpsAltitude));
	}

	if (missionStage == climb && altitudeOfApogee != -1) {
		missionStage = apogee;
		Serial.println("[Stage] Apogee Detected at " + String(altitudeOfApogee));
	}

	if (missionStage == apogee && altitudeTrend < 0 && altitudePerMinute <= altitudeLossPerMinuteForDescentDetection) {
		missionStage = descent;
		Serial.println("[Stage] Descent Detected at " + String(gpsAltitude));
	}

	if ((missionStage == descent && altitudePerMinute < 2) && (gpsAltitude <= minimumAltitudeToTriggerRecovery)) {
		recoveryDetectionIterations++;
		if (recoveryDetectionIterations >= iterationsInLowDescentToTriggerRecovery) {			
			missionStage = recovery;
			Serial.println("[Stage] Recovery Detected at " + String(gpsAltitude));	
		}
	} 


	
	
}

//Helper Functions
String satString() {	
  return String(gpsParser.time.value()) + "," + 
  String(gpsParser.location.lat(), 4) + "," + 
  String(gpsParser.location.lng(), 4) + "," + 
  String(gpsParser.altitude.feet(),0) + "," + 
  String(gpsParser.speed.knots(),0) + "," +
   String(gpsParser.course.deg(),0) + "," + 
   String(gpsParser.satellites.value()) + "," + 
   String(gpsParser.hdop.value()) +  "," +    
   String(batteryLevel/10,0) +  "," + 
   missionStageShortString();

}

String missionStageShortString() {
	if (missionStage == ground)  { return "G"; }
	if (missionStage == climb)  { return "C"; }
	if (missionStage == apogee)  { return "A"; }
	if (missionStage == descent)  { return "D"; }
	if (missionStage == recovery)  { return "R"; }
}

void setupBatteryCharger() {
	PMIC pmic; //Initalize the PMIC class so you can call the Power Management functions below. 
	pmic.setChargeCurrent(0,0,1,0,0,0); //Set charging current to 1024mA (512 + 512 offset)
	pmic.setInputCurrentLimit(2000);
}


// LISTENING EVENTS
void GPSEvent()
{	
	if (GPS.available()) {
		while (GPS.available()) {			
			char c = GPS.read();			
			gpsParser.encode(c);
			if (gpsDebugDump==true) {
				Serial.print(c);  
			}
			
		}
	}	

	if (gpsParser.hdop.value() < 300) {
		gpsState = Fix;		
			if (initialGPSAltitude==-1 && gpsParser.altitude.feet() > 0) {
				initialGPSAltitude = gpsParser.altitude.feet();	
				Serial.println("[Event] Initial Altitude Set to: " + String(initialGPSAltitude,0));
			}
	}	else {
		gpsState = noFix;		
	}

}

void computerEvent() 
{
	if (COMPUTER.available()) {
		while (COMPUTER.available()) {			
			char c = COMPUTER.read();
			computerSerialData = computerSerialData + c;
			if (c == '\n') {
				computerSerialData = computerSerialData.remove(computerSerialData.indexOf('\n'));
				computerRequest(computerSerialData);
				computerSerialData = "";
			}
		}
	}	
}

int computerRequest(String param) {	
	
	if  (param == "deboff") {
		debugMode = false;
		COMPUTER.println("OK");
		return 1;
	}
	if  (param == "debon") {
		debugMode = true;
		COMPUTER.println("OK");
		return 1;
	}
	if  (param == "simon") {
		simulationMode = true;
		COMPUTER.println("OK");
		return 1;
	}
	if  (param == "simoff") {
		simulationMode = false;
		COMPUTER.println("OK");
		return 1;
	}
	if  (param == "reset") {
		simulationMode = false;
		missionStage = ground;
		initialGPSAltitude = -1.0;
		lastPositiveGPSAltitude = 0.0;
		lastGPSAltitude = -1.0;
		altitudePerMinute = 0.0;
		altitudeOfApogee = -1.0;
		simulatedAltitude = 0.0; //For simulation only.
		COMPUTER.println("OK");
		return 1;
	}
	if (param == "reboot") {
		System.reset();		
	}
	if (param == "cellon") {		
		COMPUTER.println("OK");
		setCellModem(true);		
		return 1;
	}
	if (param == "celloff") {		
		setCellModem(false);
		return 1;
	}
	if (param == "gpsdump") {		
		gpsDebugDump = !gpsDebugDump;
		return 1;
	}
	if  (param == "vsi?") {
		COMPUTER.println(String(altitudePerMinute));		
	}
	if  (param == "alt?") {
		COMPUTER.println(String(lastGPSAltitude));		
	}
	if  (param == "apogee?") {
		COMPUTER.println(String(altitudeOfApogee));
	}
	if  (param == "stage?") {
		COMPUTER.println(missionStageShortString());
	}	
	if  (param == "cell?") {
		if (Cellular.ready() == true) { 
			COMPUTER.println("YES");
		} else {
			COMPUTER.println("NO");
		}
	}	
	if  (param == "cellconnecting?") {
		if (Cellular.connecting() == true) { 
			COMPUTER.println("YES");
		} else {
			COMPUTER.println("NO");
		}
	}
	if  (param == "rssi?") {
		CellularSignal sig = Cellular.RSSI();		
		Serial.println(sig);

	}	

	if  (param == "cloud?") {
		if (Particle.connected() == true) { 
			COMPUTER.println("YES");
		} else {
			COMPUTER.println("NO");
		}
	}	

	if (param == "fwversion?") {
		COMPUTER.printlnf("Firmware version: %s", System.version().c_str());
	}

	if (param == "?") {
		COMPUTER.printlnf("-------------------------.--------------------------");		
		COMPUTER.printlnf("Status Sentence (1hz):");		
		COMPUTER.printlnf("LAT,LON,ALT,SPEED,COURSE,SATS,HDOP,BATT,STAGE");		
		COMPUTER.printlnf("-------------------------.--------------------------");		
		COMPUTER.printlnf("deboff = Debug Off");
		COMPUTER.printlnf("debon = Debug On");
		COMPUTER.printlnf("simon = Start Simulation");
		COMPUTER.printlnf("simoff = Stop Simulation");
		COMPUTER.printlnf("reset = Set mission to ground mode");
		COMPUTER.printlnf("reboot = Reboot Flight Computer");
		COMPUTER.printlnf("simon = Start Simulation");
		COMPUTER.printlnf("cellon = Cell Modem On");
		COMPUTER.printlnf("celloff = Cell Modem Off");
		COMPUTER.printlnf("celloff = Cell Modem Off");
		COMPUTER.printlnf("gpsdump = GPS Serial Dump to computer toggle");
		COMPUTER.printlnf("vsi? = Vertical Speed?");
		COMPUTER.printlnf("alt? = Altitude in feet?");
		COMPUTER.printlnf("cell? = Cell Status?");
		COMPUTER.printlnf("cellconnecting? = Cell Modem attempting to connect?");
		COMPUTER.printlnf("rssi? = Cell Signal Strength?");
		COMPUTER.printlnf("cloud? = Is cloud available?");
		COMPUTER.printlnf("fwversion? = OS Firmware Version?");		
		COMPUTER.printlnf("-------------------------.--------------------------");
	}

	return 0;
}

void setCellModem(bool value) {		
	cellModemEnabled = value;
	if ((cellModemEnabled == true) && (Cellular.ready() == false) && (Cellular.connecting() == false)) {
		Cellular.on();
		Cellular.connect();
		Particle.connect();
		Serial.println("[Event] Cell Modem ON");
		// waitUntil(Particle.connected);
		// Serial.println("[Event] Cloud ONLINE.");
	} else if (cellModemEnabled == false) {
		Cellular.off();
		Serial.println("[Event] Cell Modem OFF");
	}
}
