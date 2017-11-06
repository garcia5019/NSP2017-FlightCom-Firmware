/*
 * Project NSP-HLM1
 * Description: Near Space Program @ ASFM
 * Author: NSP Team
 * Date: October/2017
 */

#include "TinyGPS++.h"
#include "Serial5/Serial5.h"

//MISSION SETTINGS
bool cellModemEnabled = true;
bool satModemEnabled = false;

float altitudeGainClimbTrigger = 20;
float altitudePerMinuteGainClimbTrigger = 5;
float altitudeLossPerMinuteForDescentDetection = 20;
float iterationsInLowDescentToTriggerRecovery = 10;
float minimumAltitudeToTriggerRecovery = 2300;


//SIM SETTINGS
bool simulationMode = false;
bool debugMode = true;
bool gpsDebugDump = false;
bool satDebugDump = false;
float simulatedApogeeAltitude = 1000;
//


SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

#define GPS Serial1
#define COMPUTER Serial
#define GPSEvent serialEvent1
#define computerEvent serialEvent
#define SATCOM Serial5
#define SATCOMEvent serialEvent5

#define SATCOMEnablePin D2

Timer timer(1000, second_tick);
int elapsedSeconds = 0;

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

int satcomSignal = -1;
bool satcomAlive = false;

int cellSignalRSSI = -1;
int cellSignalQuality = -1;

String computerSerialData;
String satSerialData;


enum MissionStage { 
	ground,
	climb,	
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
	//TAKE CONTROL OF THE RGB LED ONBOARD
	RGB.control(true); 
	//Setup SATCOM
	pinMode(SATCOMEnablePin, OUTPUT);	
	digitalWrite(SATCOMEnablePin, HIGH);
	//CONNECT TO GPS
	GPS.begin(57600);
	//CONNECT TO COMPUTER VIA USB
	COMPUTER.begin(57600);	
	//SETUP BATTERY CHARGER 
	setupBatteryCharger();	
	//CHANGE LED COLOR
	RGB.color(0, 0, 255);
	RGB.brightness(100);	
	//Set Cell Modem depending on the initial setup configuration (See CellModemEnable var at start of code)
	setCellModem(cellModemEnabled);
	//WAIT 2 seconds to start
	delay(2000);
	RGB.color(255, 255, 255);
	RGB.brightness(0);
	//BEGIN SatCom Comunications (delay further to allow super cap to charge);
	SATCOM.begin(19200, SERIAL_8N1);
	setSatModem(satModemEnabled);
	//Start the basic timed events
	timer.start();	
	//READ INITIAL POWER LEFT In Battery
	batteryLevel = fuel.getSoC();	
	sendToComputer("[Stage] Ground ");	

}

void loop() {} //


//TIMED EVENTS
//This will happen every second.
void second_tick() {
	timer.stopFromISR();
	elapsedSeconds++;
	
	if (missionStage == ground && Cellular.ready() == false) { RGB.color(255,165,0); } //Orange
	if (missionStage == ground && Cellular.connecting() == true) { RGB.color(50,50,255); } //Whitish
	if (missionStage == ground && Cellular.ready() == true) { RGB.color(0,255,0); } //Green
	if (missionStage == climb) { RGB.color(0,0,255); } //Blue
	if (missionStage == descent) { RGB.color(255,215,0); } //Yellow
	if (missionStage == recovery) { RGB.color(255,0,255); } //Magenta

	if (elapsedSeconds % 2 == 0 ) { //Pulse LED every 2 seconds		
		RGB.brightness(0);
	} else {
		if (debugMode == true) {
			RGB.brightness(100);
		}
	}
	

	if (elapsedSeconds % 9 == 0) {		
		if (satModemEnabled == true && satcomAlive == false) {			
			SatPing();
		}

		if (satModemEnabled == true && satcomAlive == true) {			
			getSatSignal();	
		}			
	}


	if (elapsedSeconds % 32 == 0) {
		getSatSignal();	
		sendStatusToCloud(); //Send SAT STRING to cloud if connected..		
	}

	if (elapsedSeconds % 45 == 0) {		
		sendStatusToSat();	
		batteryLevel = fuel.getSoC(); 		
		elapsedSeconds = 0;
	}
 	

	//Second Updates////////////	(State machine manager will update the stage)
	updateStage();
	/////////////////

	if (debugMode == true && gpsDebugDump == false && satDebugDump == false) {		
		sendToComputer(satString());		
	}

	

	timer.startFromISR();		
}


//State Updater
void updateStage() {

	float gpsAltitude = gpsParser.altitude.feet() + simulatedAltitude;	
	float altitudeGain = (gpsAltitude - initialGPSAltitude);
	float altitudeTrend = gpsAltitude - lastGPSAltitude;	
	altitudePerMinute = altitudeTrend * 60;
	
	if ((gpsAltitude < lastGPSAltitude) && (gpsAltitude > altitudeOfApogee) && lastGPSAltitude != -1)  {		
		sendToComputer("[Event] Apogee Reached at " + String(gpsAltitude));
		
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
		sendToComputer("[Stage] Climb Detected at: " + String(gpsAltitude));		
	}


	if (missionStage == climb && altitudeTrend < 10 && altitudePerMinute <= altitudeLossPerMinuteForDescentDetection) {
		missionStage = descent;		
		sendToComputer("[Stage] Descent Detected at " + String(gpsAltitude));
	
	}

	if ((missionStage == descent && altitudePerMinute < 2) && (gpsAltitude <= minimumAltitudeToTriggerRecovery)) {
		recoveryDetectionIterations++;
		if (recoveryDetectionIterations >= iterationsInLowDescentToTriggerRecovery) {			
			missionStage = recovery;			
			sendToComputer("[Stage] Recovery Detected at " + String(gpsAltitude));				
		}
	} 


	
	
}

void sendStatusToCloud() {
	if (Particle.connected() == true) { 		
		Particle.publish("S",satString());		
	}
}

void sendStatusToSat() {	
	sendTextToSat(satString());
}

//Helper Functions
String satString() {	
	//GPSTIme:
  return String(gpsParser.time.value()) + "," + 
  String(gpsParser.location.lat(), 4) + "," + 
  String(gpsParser.location.lng(), 4) + "," + 
  String(gpsParser.altitude.feet(),0) + "," + 
  String(gpsParser.speed.knots(),0) + "," +
   String(gpsParser.course.deg(),0) + "," + 
   String(gpsParser.satellites.value()) + "," + 
   String(gpsParser.hdop.value()) +  "," +    
   String(batteryLevel/10,0) +  "," + 
   String(satcomSignal) +  "," +    
   missionStageShortString();

}

String missionStageShortString() {
	if (missionStage == ground)  { return "G"; }
	if (missionStage == climb)  { return "C"; }	
	if (missionStage == descent)  { return "D"; }
	if (missionStage == recovery)  { return "R"; }
}

void setupBatteryCharger() {
	PMIC pmic; //Initalize the PMIC class so you can call the Power Management functions below. 
	pmic.setChargeCurrent(0,0,1,0,0,0); //Set charging current to 1024mA (512 + 512 offset)
	pmic.setInputCurrentLimit(2000);
}


void SATCOMEvent() {	
		while (SATCOM.available()) {			
			char c = SATCOM.read();	
			satSerialData = satSerialData + c;								
			if (satDebugDump==true && gpsDebugDump==false) {
				TRY_LOCK(COMPUTER) {
					COMPUTER.write(c);
				}
			}
			if (c == '\r') {				
				satSerialData = satSerialData.trim();
				if (satSerialData.substring(0,2) == "OK" && satcomAlive == false) {
					satcomAlive = true;					
					sendToComputer("[Event] SatCom Alive");					
					getSatSignal();
				}								
				if (satSerialData.substring(0,5) == "+CSQ:") {
					String signal = satSerialData.substring(5);
					satcomSignal = signal.toInt();										
				}
				
				satSerialData = "";
			}
		}	
}

// LISTENING EVENTS
void GPSEvent()
{	
	if (GPS.available()) {
		while (GPS.available()) {			
			char c = GPS.read();			
			gpsParser.encode(c);
			if (gpsDebugDump==true && satDebugDump==false) {
				TRY_LOCK(COMPUTER) {
					COMPUTER.print(c);  
				}
			}			
		}
	}	

	if (gpsParser.hdop.value() < 300) {
		gpsState = Fix;		
			if (initialGPSAltitude==-1 && gpsParser.altitude.feet() > 0) {
				initialGPSAltitude = gpsParser.altitude.feet();					
				sendToComputer("[Event] Initial Altitude Set to: " + String(initialGPSAltitude,0));				
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
		sendToComputer("OK");
		return 1;
	}
	if  (param == "debon") {
		debugMode = true;
		sendToComputer("OK");
		return 1;
	}
	if  (param == "simon") {
		simulationMode = true;
		sendToComputer("OK");
		return 1;
	}
	if  (param == "simoff") {
		simulationMode = false;
		sendToComputer("OK");
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
		sendToComputer("OK");
		return 1;
	}
	if (param == "reboot") {
		sendToComputer("OK");
		System.reset();		
	}
	if (param == "cellon") {				
		setCellModem(true);		
		return 1;
	}
	if (param == "celloff") {		
		setCellModem(false);
		return 1;
	}
	if (param == "saton") {				
		setSatModem(true);		
		return 1;
	}
	if (param == "satoff") {		
		setSatModem(false);
		return 1;
	}
	if (param == "comon") {				
		setSatModem(true);	
		setCellModem(true);	
		return 1;
	}
	if (param == "comoff") {		
		setSatModem(false);
		setCellModem(false);
		return 1;
	}
	if (param == "gpsdump") {		
		gpsDebugDump = !gpsDebugDump;
		return 1;
	}
	if (param == "satdump") {		
		satDebugDump = !satDebugDump;
		return 1;
	}
	if (param == "querysatsignal") {		
		sendToComputer("OK");
		getSatSignal();
		return 1;
	}	
	if (param == "querycellsignal") {		
		sendToComputer("OK");
		getCellSignal();
		return 1;
	}
	if  (param == "vsi?") {
		sendToComputer(String(altitudePerMinute));		
	}
	if  (param == "alt?") {
		sendToComputer(String(lastGPSAltitude));		
	}
	if  (param == "apogee?") {
		sendToComputer(String(altitudeOfApogee));
	}
	if  (param == "stage?") {
		sendToComputer(missionStageShortString());
	}	
	if  (param == "cell?") {
		if (Cellular.ready() == true) { 
			sendToComputer("YES");
		} else {
			sendToComputer("NO");
		}
	}	
	if  (param == "cellconnecting?") {
		if (Cellular.connecting() == true) { 
			sendToComputer("YES");
		} else {
			sendToComputer("NO");
		}
	}
	if  (param == "cellsignal?") {		
		CellularSignal sig = Cellular.RSSI();		
		sendToComputer(sig);
	}	

	if  (param == "cloud?") {
		if (Particle.connected() == true) { 
			sendToComputer("YES");
		} else {
			sendToComputer("NO");
		}
	}	
	
	if  (param == "satsignal?") {
		sendToComputer(String(satcomSignal));
		return satcomSignal;
	}	

	if  (param == "satenabled?") {
		if (satModemEnabled) {
			sendToComputer("YES");
			return 1;
		}
		sendToComputer("NO");
		return false;
	}

	if (param == "fwversion?") {
		TRY_LOCK(COMPUTER) {
			COMPUTER.printlnf("Firmware version: %s", System.version().c_str());
		}
	}

	if (param == "$") {
		sendToComputer(satString());		
	}

	if (param == "$$") {	
		sendToComputer("OK");	
		sendStatusToCloud();		
	}

	if (param == "$$$") {	
		sendToComputer("OK");
		sendStatusToSat();		
	}

	

	if (param == "?") {
		TRY_LOCK(COMPUTER) {
		COMPUTER.println("-------------------------.--------------------------");		
		COMPUTER.println("Status Sentence (1hz):");
		COMPUTER.println("TIME,LAT,LON,ALT,SPEED,COURSE,SATS,HDOP,BATT,SAT,STAGE");
		COMPUTER.println("-------------------------.--------------------------");		
		COMPUTER.println("deboff = Debug Off");
		COMPUTER.println("debon = Debug On");
		COMPUTER.println("simon = Start Simulation");
		COMPUTER.println("simoff = Stop Simulation");
		COMPUTER.println("reset = Set mission to ground mode");
		COMPUTER.println("reboot = Reboot Flight Computer");
		COMPUTER.println("simon = Start Simulation");
		COMPUTER.println("cellon = Cell Modem On");
		COMPUTER.println("celloff = Cell Modem Off");
		COMPUTER.println("saton = SAT Modem ON");
		COMPUTER.println("satoff = SAT Modem Off");
		COMPUTER.println("comoff = All Comunication systems OFF [cell + sat]");
		COMPUTER.println("comon = All Comunication systems ON [cell + sat]");
		COMPUTER.println("gpsdump = GPS Serial Dump to computer toggle");
		COMPUTER.println("satdump = SATCOM Serial Dump to computer toggle");
		COMPUTER.println("querysatsignal = Send a request to the satelite modem to get sat signal");
		COMPUTER.println("querycellsignal = Send a request to the cellular modem to get RSSI signal");
		COMPUTER.println("vsi? = Vertical Speed?");
		COMPUTER.println("alt? = Altitude in feet?");
		COMPUTER.println("cell? = Cell Status?");
		COMPUTER.println("cellconnecting? = Cell Modem attempting to connect?");
		COMPUTER.println("cellsignal? = Cell Signal Strength [RSSI,QUAL] ?");
		COMPUTER.println("cloud? = Is cloud available?");
		COMPUTER.println("satsignal? = 0-5 Satcom signal strength?");		
		COMPUTER.println("satenabled? = Is the sat modem enabled?");		
		COMPUTER.println("fwversion? = OS Firmware Version?");		
		COMPUTER.println("$ = Print status string");		
		COMPUTER.println("$$ = Print and send to cell cloud status string");		
		COMPUTER.println("$$$ = Print and send to SAT cloud status string");		
		COMPUTER.println("-------------------------.--------------------------");
		}

	}

	return 0;
}

void setCellModem(bool value) {		
	cellModemEnabled = value;
	if ((cellModemEnabled == true) && (Cellular.ready() == false) && (Cellular.connecting() == false)) {		
			Cellular.on();
			Cellular.connect();
			Particle.connect();					
		sendToComputer("[Event] Cell Modem ON");
		getCellSignal();		
	} else if (cellModemEnabled == false) {
		Cellular.off();
		cellSignalRSSI = -1;
		cellSignalQuality = -1;
		sendToComputer("[Event] Cell Modem OFF");
	}
}

void getCellSignal() {
	if (cellModemEnabled == false) { return; }		
		CellularSignal cellSignal = Cellular.RSSI();
		cellSignalRSSI = cellSignal.rssi;
		cellSignalQuality = cellSignal.qual;
}

void setSatModem(bool value) {	
	if (satModemEnabled == false && value == true) {  sendToComputer("[Event] SAT Modem ON"); }
	satModemEnabled = value;	
	if (satModemEnabled == false) {   satcomAlive = false; satcomSignal = -1; sendToComputer("[Event] SAT Modem OFF"); return; }	
	SATCOM.println("AT&K0");
	getSatSignal();	
}

void getSatSignal() {
	if (satModemEnabled == false) { return; }				
		SATCOM.println("AT+CSQ");	
}

void sendTextToSat(String text) {
	if (satModemEnabled == false) { return; }			
		SATCOM.println("AT+SBDWT=" + text);	
}

void SatPing() {	
		SATCOM.println("AT");	
}

void sendToComputer(String text) {
	TRY_LOCK(COMPUTER) {	
		COMPUTER.println(text);
	}
}



