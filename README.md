# NSP2017-FlightCom-Firmware
ASFM Near Space Program 2017 - Master Flight Computer Firmware for HLM1 Capsule

## Welcome to NSP2017-FlightCom-Firmware Master Flight Computer Firmware (MFC)
The flight computer consists of a *Particle Electron* that serves as the primary MCU. This device lives in the Master Flith Computer PCB (https://github.com/ASFM-HLM-STEMLAB/NSP2017-FlightCom-Hardware).

The MFC is responsible of telemetry (SAT+CELL), navigation (GPS), logging and power management. It then comunicates via i2C with the environmental sensors. The MFC contains an onboard temperature sensor to monitori vital signs and to trigger a heater should the temperature get too cold for the battery and electronics. It is also in charge of comunicating to ground control thru either the onboard cellular modem or the external RockBlock Iridium Modem.

#### Project Structure 
1.- ```/FlightComputer``` folder: Contains the firmware source code for the master flight computer (particle).
2.- ```/SlaveSensors``` folder: Contains the SlaveSensor source code for the slave sensors to send data to the FC.

Each folder contains a ```/src``` folder which holds the src code for the project.

Item 1.- Should be compiled with Particle build tools for the Electron device.
Item 2.- Should be compiled with Arduino unless otherwise specified.


#### ```/src``` folder:  
This is the source folder that contains the firmware files for your project. It should *not* be renamed. 
Anything that is in this folder when you compile your project will be sent to our compile service and compiled into a firmware binary for the Particle device that you have targeted.

If your application contains multiple files, they should all be included in the `src` folder. If your firmware depends on Particle libraries, those dependencies are specified in the `project.properties` file referenced below.

#### ```.ino``` file:
This file is the firmware that will run as the primary application on your Particle device. It contains a `setup()` and `loop()` function, and can be written in Wiring or C/C++. For more information about using the Particle firmware API to create firmware for your Particle device, refer to the [Firmware Reference](https://docs.particle.io/reference/firmware/) section of the Particle documentation.

#### ```project.properties``` file:  
This is the file that specifies the name and version number of the libraries that your project depends on. Dependencies are added automatically to your `project.properties` file when you add a library to a project using the `particle library add` command in the CLI or add a library in the Desktop IDE. (Only Applies to Particle projects)

## Compiling (Particle)
Get the Particle CLI program.

When you're ready to compile your project, make sure you have the correct Particle device target selected and run `particle compile <platform>` in the CLI or click the Compile button in the Desktop IDE. The following files in your project folder will be sent to the compile service:

- Everything in the `/src` folder, including your `.ino` application file
- The `project.properties` file for your project
- Any libraries stored under `lib/<libraryname>/src`
=======
