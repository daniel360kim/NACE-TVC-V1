/*
The NACE flight computer logs data continuously to a microSD card. These functions setup and utilize the microSD card for datalogging
*/

#ifndef SDcard_h
#define SDcard_h
#include <Arduino.h>
#include <SD.h> //built in SD libary with arduino
#include <Settings.h>
#include <Outputs.h>
#include <Navigation.h>

char comma[3] = ","; //we use char because arduino has trouble with strings, furthermore, benchmark tests show that using a character is faster

//Initializing file names 
File file;
#define FILE_BASE_NAME_DATA "Data"
const uint8_t BASE_NAME_SIZE_DATA = sizeof(FILE_BASE_NAME_DATA) - 1;
char fileNamedata[] = FILE_BASE_NAME_DATA "00.csv";
 
#define FILE_BASE_NAME_STARTUP "START"
const int BASE_NAME_SIZE_STARTUP = sizeof(FILE_BASE_NAME_STARTUP) - 1;
char fileNameStartup[] = FILE_BASE_NAME_STARTUP "00.txt";

//used for interval logging
unsigned long previousSDlog;

//SD chip select pinout
const uint8_t SDchipSelect = 3; //fill this out with CS GPIO pin

void errorCheck(){ //checks to see if there are any errors when creating the file name
 if (!file) {
 Serial.println(F(fileNamedata));
 Serial.print(F(" open failed"));
 indicateError();
  }
}

//this startup file logs important settings and headers for the data that can be utilized later on
void createStartupFile(){
 file = SD.open(fileNamedata, FILE_WRITE);
 errorCheck();
  //headers for the csv file that is being made
  file.print("void,state,fliTime,rPres,rAlti,Pre,Alti,rax,ray,raz,ax,ay,az,wfXa,wfYa,wfZa,Vx,Vy,Vz,Px,Py,Pz,fgx,fgy,fgz,X,Y,Z,ErrorX,ErrorY,PIDX,PIDY");
  file.close();
 
  file = SD.open(fileNameStartup, FILE_WRITE);
  errorCheck();
  
  //details of the flight computer's settings
  file.println("NACE Euler Edition V2: TVC Static/Flight Software V3.4");
  file.println("Current File Names:");
  file.println(fileNameStartup);
  file.println(fileNamedata);
  file.println("PID Values:");
  file.println(kp);
  file.println(ki);
  file.println(kd);
  file.println("Accel: ODR:400HZ - BW:40HZ//Range 12G");
  file.println("Gyro: ODR:400HZ - BW:47HZ//Range 2000DPS");
  file.println("Barometer OS:X8");
  file.println("Temperature OS:X2");
  file.close();
}

void initializeSD(){ //self explanatory
    if (!SD.begin(SDchipSelect)) { //determine if SD card has any errors
    while(1){
      Serial.println(F("SD Card Error"));
      buzzer.begin(0);
      indicateError();
    }
  }
 //these two loops create a new file name based on previous files so we dont have to delete the file every time we run the computer for instance,
 //if we already have data01.csv, these loops will set the name to data02.csv
  while (SD.exists(fileNamedata)) {
    if (fileNamedata[BASE_NAME_SIZE_DATA + 1] != '9') {
      fileNamedata[BASE_NAME_SIZE_DATA + 1]++;
    } else if (fileNamedata[BASE_NAME_SIZE_DATA] != '9') {
      fileNamedata[BASE_NAME_SIZE_DATA + 1] = '0';
      fileNamedata[BASE_NAME_SIZE_DATA]++;
    } else {
      Serial.println(F("Can't create file name"));
      indicateError();
 
    }
  }
 
  while (SD.exists(fileNameStartup)) { 
    if (fileNameStartup[BASE_NAME_SIZE_STARTUP + 1] != '9') {
      fileNameStartup[BASE_NAME_SIZE_STARTUP + 1]++;
    } else if (fileNameStartup[BASE_NAME_SIZE_STARTUP] != '9') {
      fileNameStartup[BASE_NAME_SIZE_STARTUP + 1] = '0';
      fileNameStartup[BASE_NAME_SIZE_STARTUP]++;
    } else {
      Serial.println(F("Can't create file name"));
      indicateError();
       
    }
  }

 createStartupFile();

 //opening a file for logging. the O_WRITE O_CREAT flags reduce latency just a bit
 file = SD.open(fileNamedata, O_WRITE | O_CREAT);
  if(!file){
    Serial.println(F(fileNamedata));
    Serial.print(F(" open failed"));
    while(1){
      indicateError();
    }
  }
}

void logData(){ //chonker function that logs the data - open at your own risk...
    file.print("\n");
    file.print(F(comma));
    file.print(systemState);
    file.print(F(comma));
    file.print(flight_time);
    file.print(F(comma));
    file.print(data.rpressure);
    file.print(F(comma));
    file.print(data.raltitude);
    file.print(F(comma));
    file.print(data.temp);
    file.print(data.pressure);
    file.print(F(comma));
    file.print(data.altitude);
    file.print(F(comma));
    file.print(data.rax);
    file.print(F(comma));
    file.print(data.ray);
    file.print(F(comma));
    file.print(data.raz);
    file.print(F(comma));
    file.print(data.ax);
    file.print(F(comma));
    file.print(data.ay);
    file.print(F(comma));
    file.print(data.az);
    file.print(F(comma));
    file.print(data.wfXa);
    file.print(F(comma));
    file.print(data.wfYa);
    file.print(F(comma));
    file.print(data.wfZa);
    file.print(F(comma));
    file.print(data.Vx);
    file.print(F(comma));
    file.print(data.Vy);
    file.print(F(comma));
    file.print(data.Vz);
    file.print(F(comma));
    file.print(data.px);
    file.print(F(comma));
    file.print(data.py);
    file.print(F(comma));
    file.print(data.pz);
    file.print(data.gx);
    file.print(F(comma));
    file.print(data.gy);
    file.print(F(comma));
    file.print(data.gz);
    file.print(F(comma));
    file.print(data.X);
    file.print(F(comma));
    file.print(data.Y);
    file.print(F(comma));
    file.print(data.Z);
    file.print(F(comma));
    file.print(data.errorX);
    file.print(F(comma));
    file.print(data.errorY);
    file.print(F(comma));
    file.print(data.PIDX);
    file.print(F(comma));
    file.print(data.PIDY);
    file.print(F(comma));

}

void slowDataLog(){ //logs data at a slow rate to keep loop time fast and to make data management easier (for launchpad idle, landed etc.)
 unsigned long currentMicros = micros();
 if(currentMicros - previousSDlog >= slowlogPeriodMicros){
   logData();
}
}

void fastDataLog(){ //logs data at a fast rate like during flight
 unsigned long currentMicros = micros();
 if(currentMicros - previousSDlog >= fastlogPeriodMicros){
   logData();
}
}

void closeFile(){ //saves the data 
  file.flush();
  file.close();
}

void fileCloseandOpen(){ //saves the file then reopens it
  closeFile();
  file = SD.open(fileNamedata, O_WRITE | O_CREAT);
}

#endif