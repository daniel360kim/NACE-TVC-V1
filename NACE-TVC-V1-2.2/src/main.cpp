#include <Arduino.h>
#include <Servo.h>
#include <Outputs.h>
#include <Navigation.h>
#include <SDcard.h>
#include <PID.h>

void setup() { //runs once at the beginning
 initializeOutputs();
 startupSequence();
 initializeSensors();
 initializeSD();
 indicateCompleteStartup();
 previous_time = micros(); //allows for smooth data and acc. pos/vel measurements at the beginning 
 systemState = 0;
}

void loop() { //runs continuously after setup

//calculating dt which is used very frequently throughout the different headers
 unsigned long microseconds = micros();
 dt = (microseconds - previous_time) / 1000000.f;
 previous_time = microseconds;

 readSensors(); //collect sensor data
 FlightTime(); //run timekeeper

 ////////States Machine////////////////
 if(systemState == 0){ //launchpad idle/ground startup
     allPyrosLow();
     slowDataLog();
     displaySpectrum();
     //errorCheckIMU(); //exit condition 1
     detectNavReady(); //exit condition 2
     Serial.println("SS: 0");
 }
 
 if(systemState == 1){ //waiting for a launch
     allPyrosLow();
     slowDataLog();
     detectLaunch(); //exit condition
     displaySpectrum();
 }
 if(systemState == 2){//powered flight
     runPID();
     Flight_Termination_Handler();
     fastDataLog();
     setColor(0,255,238); //sky blue
     detectBurnout(); //exit condition
   
 }
 else if(systemState == 3){ //unpowered flight
     runPID();
     fastDataLog();
     setColor(255,213,0); //yellow
     detectApogee(); //exit condition
     
 }
 else if(systemState == 4){ //descent
     detachServos();
     fileCloseandOpen(); //saves data in case of corruption or loss of data on "landing" ascent is mostly the data we need, but descent is usefull too
     fastDataLog();
     setColor(0,255,60); //spring green
     detectLanding();

 }
 else if(systemState == 5){ //landed
     indicateLanding();
     slowDataLog(); //although landing was detected, we slow datalog in case of mistriggered exit state 

 }


 if(systemState == 6){//abort called
     closeFile();
     setColor(255,0,0);

 }
    Serial.println(data.altitude);
}
