#include <Arduino.h>
#include "BMI088.h"   
#include <math.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>
#include <BMP388_DEV.h>
#include <Buzzer.h>
#include <SD.h>

File file;
#define FILE_BASE_NAME_DATA "Data" 
const uint8_t BASE_NAME_SIZE_DATA = sizeof(FILE_BASE_NAME_DATA) - 1;
char fileNamedata[] = FILE_BASE_NAME_DATA "00.csv";

#define FILE_BASE_NAME_STARTUP "START"
const int BASE_NAME_SIZE_STARTUP = sizeof(FILE_BASE_NAME_STARTUP) - 1;
char fileNameStartup[] = FILE_BASE_NAME_STARTUP "00.txt";

Buzzer buzzer(5);

BMP388_DEV bmp388;
float temp, altitude, flightAltitude, groundAltitude, pressure;
SimpleKalmanFilter KalmanFilterX(2.123, 2, 1);
SimpleKalmanFilter KalmanFilterY(1.98, 2, 1);
SimpleKalmanFilter KalmanFilterZ(1.98, 2, 1);
/*
SimpleKalmanFilter KalmanFilterGx(0.5, 0.5, 1);
SimpleKalmanFilter KalmanFilterGy(0.5, 0.5, 1);
SimpleKalmanFilter KalmanFilterGz(0.5, 0.5, 1);
*/
Servo servoX;
Servo servoY;

float a[4] = {1, 0, 0, 0};
float q[4] = {1, 0, 0, 0};
float b[4] = {1, 0, 0, 0};
float ori_acc_prime[4];
float ori_acc[4];
float quats_prime[4];
float X, Y, Z;

float PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY;

//PID Gains//
float kp = .6;
float ki = .1;
float kd = 0.05;

bool stateMachine_enable = false;
bool abortFunctionality_enable = false;

Bmi088Accel accel(Wire,0x18);
Bmi088Gyro gyro(Wire,0x68);

float previous_time, dt;
float abort_time, flight_time, flight_starttime;
uint8_t FTstCollected = 0;

float bmpIntervalPrevious, bmpIntervalCurrent, intervalAltitude;
float apogeeIntervalCurrent, apogeeIntervalPrevious;
float pyroTime;

float gxBias, gyBias, gzBias;
float axBias, ayBias, azBias;

int systemState = 0;
bool abort_armed = false;

//Upright Angle of the Gyroscope
const uint8_t desired_angleX = 0;//servoY
const uint8_t desired_angleY = 0;//servoX
//Offsets for tuning 
const int32_t servoY_offset = 112;
const int servoX_offset = 39;
//Position of servos through the startup function
int servoXstart = servoY_offset * -1;
int servoYstart = servoX_offset;
//Ratio between servo gear and tvc mount
const uint8_t servoX_gear_ratio = 2;
const uint8_t servoY_gear_ratio = 2;
//"PID" Constants
float X_p = 0;
float Y_p = 0;
float Y_i = 0;
float X_i = 0;
float X_d = 0;
float Y_d = 0;

const uint8_t redLED = 8;
const uint8_t grnLED = 18;
const uint8_t bluLED = 9;
const uint8_t pyroCont = A2;
const uint8_t pyro = 4;
const uint8_t SDchipSelect = 3; //fill this out with CS GPIO pin

void calculateDrift(){
  float gxIntegration;
  float gyIntegration;
  float gzIntegration;

  float axIntegration;
  float ayIntegration;
  float azIntegration;
  
  for(int i = 0; i < 100; i++){
    gyro.readSensor();
    accel.readSensor();

    float ax = accel.getAccelX_mss();
    float ay = accel.getAccelY_mss();
    float az = accel.getAccelZ_mss();
    float gx = gyro.getGyroX_rads();
    float gy = gyro.getGyroY_rads();
    float gz = gyro.getGyroZ_rads();

    gxIntegration = gxIntegration + gx;
    gyIntegration = gyIntegration + gy;
    gzIntegration = gzIntegration + gz;

    axIntegration = axIntegration + ax;
    ayIntegration = ayIntegration + ay;
    azIntegration = azIntegration + az;
  }
  gxBias = gxIntegration / 100;
  gyBias = gyIntegration / 100;
  gzBias = gzIntegration / 100;

  axBias = axIntegration / 100;
  ayBias = ayIntegration / 100;
  azBias = azIntegration / 100;

}

void indicateError() {
  while(1){
    buzzer.begin(0);
    digitalWrite(bluLED, LOW);
    digitalWrite(grnLED, LOW);
    digitalWrite(redLED, HIGH);

    buzzer.sound (NOTE_C7, 100);

    digitalWrite(redLED, LOW);
    buzzer.end(100);
  }
}

void readVoltage(){

}

void setup() {

 pinMode(13, OUTPUT);
 pinMode(11, OUTPUT);
 pinMode(bluLED, OUTPUT);
 pinMode(grnLED, OUTPUT);
 pinMode(redLED, OUTPUT);
 pinMode(pyro, OUTPUT);
 pinMode(12, OUTPUT); 
 pinMode(A2, INPUT);
 digitalWrite(pyro, LOW);
 digitalWrite(12, HIGH);//disable auto-retry functionality on load driver

  buzzer.begin(0);
  digitalWrite(redLED, HIGH);
  buzzer.sound(NOTE_C5, 600);
  digitalWrite(redLED, LOW);
  delay(100);
  digitalWrite(grnLED, HIGH);
  buzzer.sound(NOTE_G4, 350);
  delay(100);
  digitalWrite(grnLED, LOW);
  digitalWrite(bluLED, HIGH);
  buzzer.sound(NOTE_G5, 700);
  digitalWrite(grnLED, HIGH);
  digitalWrite(redLED, HIGH);
  Serial.begin(115200);
  servoX.attach(11);
  servoY.attach(13);
  servoX.write(servoXstart);
  servoY.write(servoYstart);
    
  if (!SD.begin(SDchipSelect)) { //determine if SD card has any errors
    while(1){
      Serial.println(F("SD Card Error"));
      buzzer.begin(0);
      indicateError();
    }
  }

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

  while (SD.exists(fileNameStartup)) { //checks to see if the data file exists and renames to a different file name if so
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

//create file 
 file = SD.open(fileNamedata, FILE_WRITE);
  if (!file) {
    Serial.println(F(fileNamedata));
    Serial.print(F(" open failed"));
    indicateError(); 
  }
  
  file.print("void,state,Time,fliTime,Temp,Pres,Alti,rax,ray,raz,fax,fay,faz,rgx,rgy,rgz,fgx,fgy,fgz,X,Y,Z,PX,IX,DX,PY,IY,DY,ErrorX,ErrorY,pwmX,pwmY,PIDX,PIDY");
  file.close();

  file = SD.open(fileNameStartup, FILE_WRITE);
  if (!file) {
    Serial.println(F(fileNameStartup));
    Serial.print(F(" open failed"));
    indicateError();
  }

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

  uint8_t status;
  status = accel.begin();
  status = accel.setOdr(Bmi088Accel::ODR_400HZ_BW_40HZ);
  status = accel.setRange(Bmi088Accel::RANGE_12G);
  if(status < 0){
    Serial.println(F("Accel Initialization Error"));
    indicateError();
  
  }
  
  status = gyro.begin();
  status = gyro.setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
  status = gyro.setRange(Bmi088Gyro::RANGE_2000DPS);
  if(status < 0){
    Serial.println(F("Gyro Initialization Error"));
    indicateError();
    
  }
  int continuity = analogRead(A2); //check for continuity at GPIO 18
  if(continuity >= 250){
    Serial.println(F("No Continuity Detected"));
    indicateError();
    
  }
  //Initialize BMP388, and get ground level reference
  bmp388.begin(BMP388_I2C_ALT_ADDR);
  bmp388.setPresOversampling(OVERSAMPLING_X8);
  bmp388.setTempOversampling(OVERSAMPLING_X2);
  bmp388.setIIRFilter(IIR_FILTER_16);
  
  digitalWrite(grnLED, LOW);
  digitalWrite(bluLED, LOW);
  digitalWrite(redLED, HIGH);
  buzzer.sound(NOTE_A4, 150);
  
  calculateDrift();

  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, HIGH);

  digitalWrite(bluLED, LOW);
  digitalWrite(grnLED, LOW);
  digitalWrite(redLED, HIGH);

  buzzer.sound(NOTE_A4, 150);

  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, HIGH);

  buzzer.sound(NOTE_D5, 150);

  digitalWrite(grnLED, LOW);
  digitalWrite(bluLED, HIGH);

  buzzer.sound(NOTE_F5, 250);

  digitalWrite(bluLED, LOW);
  digitalWrite(grnLED, LOW);
  digitalWrite(redLED, LOW);
  servoX.attach(11);
  servoY.attach(13);
  servoX.write(servoXstart);
  servoY.write(servoYstart);

  file = SD.open(fileNamedata, FILE_WRITE);
  if(!file){
    Serial.println(F(fileNamedata));
    Serial.print(F(" open failed"));
    while(1){
      indicateError();
    }
  }

  systemState++; //move system to launchpad idle
}

void getScalar(float theta) {
  b[0] = cos(theta / 2);
}

void getVector(float x, float y, float z, float quatNorm, float theta) {
  b[1] = (x / quatNorm) * sin(theta / 2);
  b[2] = (y / quatNorm) * sin(theta / 2);
  b[3] = (z / quatNorm) * sin(theta / 2);
}

void abortFlight(){
  systemState = 6;
  digitalWrite(pyro, HIGH);
  servoX.detach();
  servoY.detach();

  file.close();

  digitalWrite(bluLED, HIGH);
  digitalWrite(grnLED, HIGH);
  digitalWrite(redLED, HIGH);
  while(1){
  }
}

int checkOri(float Y, float Z){
 uint8_t result = 0;
 if(Y >= 30 || Y <= -30){
   if(abort_armed == true){
     result = 1;
   }
 }

 if(X >= 30 || X <= -30){
   if(abort_armed == true){
     result = 1;
   }
 }
 return result;
}

void flightTime() {
 
 if(FTstCollected == 0){
   flight_starttime = micros();
   FTstCollected++;
    }
    
  else{
      flight_time = micros() - flight_starttime;
    }

}

void loop()
{

 dt = (micros() - previous_time) / 1000000;
 previous_time = micros();
 
 if(stateMachine_enable == false){
    flightTime();
 }
 
 accel.readSensor();
 gyro.readSensor();
 
 float rax = accel.getAccelX_mss();
 float ray = accel.getAccelY_mss();
 float raz = accel.getAccelZ_mss();
 float rgx = gyro.getGyroX_rads();
 float rgy = gyro.getGyroY_rads();
 float rgz = gyro.getGyroZ_rads();

 float ax = KalmanFilterX.updateEstimate(rax);
 float ay = KalmanFilterY.updateEstimate(ray);
 float az = KalmanFilterZ.updateEstimate(raz);

/*
 ax = ax - axBias;
 ay = ay - ayBias;
 az = az - azBias;
*/
 float gx = rgx - gxBias;
 float gy = rgy - gyBias;
 float gz = rgz - gzBias;

 float quatNorm = sqrt(sq(gx) + sq(gy) + sq(gz));
 quatNorm = max(abs(quatNorm), 1e-12);

 float theta = quatNorm * dt;

 getScalar(theta);
 getVector(gx, gy, gz, quatNorm, theta);

 a[0] = q[0];
 a[1] = q[1];
 a[2] = q[2];
 a[3] = q[3];

 q[0] = (a[0] * b[0]) - (a[1] * b[1]) - (a[2] * b[2]) - (a[3] * b[3]);
 q[1] = (a[0] * b[1]) + (a[1] * b[0]) + (a[2] * b[3]) - (a[3] * b[2]);
 q[2] = (a[0] * b[2]) - (a[1] * b[3]) + (a[2] * b[0]) + (a[3] * b[1]);
 q[3] = (a[0] * b[3]) + (a[1] * b[2]) - (a[2] * b[1]) + (a[3] * b[0]);

 quats_prime[0] = q[0];
 quats_prime[1] = -1 * q[1];
 quats_prime[2] = -1 * q[2];
 quats_prime[3] = -1 * q[3];
 
 ori_acc[0] = q[0] * 0 - q[1] * ax - q[2] * ay - q[3] * az;
 ori_acc[1] = q[0] * ax + q[1] * 0 + q[2] * az - q[3] * ay;
 ori_acc[2] = q[0] * ay - q[1] * az + q[2] * 0 + q[3] * ax;
 ori_acc[3] = q[0] * az + q[1] * ay - q[2] * ax + q[3] * 0;

 //ori_acc_prime[0] = ori_acc[0] * q[0] - ori_acc[1] * q[1] * -1 - ori_acc[2] * q[2] * -1 - ori_acc[3] * q[3] * -1; this i commented out because it will always be zero
 ori_acc_prime[1] = (ori_acc[0] * q[1] * -1 + ori_acc[1] * q[0] + ori_acc[2] * q[3] * -1 - ori_acc[3] * q[2] * -1) - 9.81;
 ori_acc_prime[2] = ori_acc[0] * q[2] * -1 - ori_acc[1] * q[3] * -1 + ori_acc[2] * q[0] + ori_acc[3] * q[1] * -1;
 ori_acc_prime[3] = ori_acc[0] * q[3] * -1 + ori_acc[1] * q[2] * -1 - ori_acc[2] * q[1] * -1 + ori_acc[3] * q[0];

 float sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
 float cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
     X = (atan2(sinr_cosp, cosr_cosp)) * RAD_TO_DEG;
  
     
 float sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
    if (abs(sinp) >= 1)
        Y = (copysignf(HALF_PI, sinp)) * RAD_TO_DEG;
    else
        Y = (asin(sinp)) * RAD_TO_DEG;
      /*  
 Y = fmod(Y, 360);
 if(Y < 0){
   Y += 360;
 }
*/

 float siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
 float cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    Z = (atan2(siny_cosp, cosy_cosp)) * RAD_TO_DEG;
 Z = fmod(Z, 360);/*
 if(Z < 0){
   Z += 360;
 }*/
 bmp388.startForcedConversion();
 bmp388.getMeasurements(temp, pressure, altitude);

 bmpIntervalCurrent = micros();
 apogeeIntervalCurrent = micros();

 const float bmpInterval = 1000000;
 const float apogeeCheck = 1500000;

 if(bmpIntervalCurrent - bmpIntervalPrevious >= bmpInterval) {
   bmpIntervalPrevious = bmpIntervalCurrent;
   intervalAltitude = altitude;
 }
 if(abortFunctionality_enable == true){
    uint8_t abort = checkOri(Y,Z);
    if(abort > 0 && abort_armed == true){
      abortFlight();
      abort = 0;
    }
 }
 if(stateMachine_enable == true){
    if(systemState == 1 && ori_acc_prime[1] >= 2){
      systemState++; //move system on to powered flight mode
      flightTime();
      abort_armed = !abort_armed;
      digitalWrite(bluLED, HIGH);
      digitalWrite(redLED, HIGH);
      digitalWrite(grnLED, HIGH);
        }
 
 
    if(systemState == 2 && ori_acc_prime[1] <= 2){
      systemState++; //move system on to unpowered coast
      flightTime();
      digitalWrite(bluLED, HIGH);
      digitalWrite(redLED, LOW);
      digitalWrite(grnLED, LOW);
        }

    if(apogeeIntervalCurrent - apogeeIntervalPrevious >= apogeeCheck && systemState == 3){
      if(intervalAltitude > altitude + 0.5){
        flightTime();
        systemState++; //move system to descent
        digitalWrite(bluLED, LOW);
        digitalWrite(grnLED, HIGH);
        digitalWrite(pyro, HIGH);
        pyroTime = micros();
      }
    }

 
    if(micros() >= pyroTime + 20000000  && systemState == 4){
        flightTime();
        systemState++; //move system on to landed
        digitalWrite(9, HIGH);//turn on all three LEDS (white)
        digitalWrite(8, HIGH);
        digitalWrite(18, HIGH);
        digitalWrite(pyro, LOW);
        buzzer.sound(NOTE_D7, 50);
        buzzer.sound(NOTE_D5, 50);
        servoX.detach();
        servoY.detach();
        //file.close();
        delay(100);
        while(1){
          buzzer.sound(NOTE_D7, 50);
          buzzer.sound(NOTE_D7, 50);
          analogWrite(redLED, random(0,255));
          analogWrite(grnLED, random(0,255));
          analogWrite(bluLED, random(0,255));
          
        }
    }
}
 else{ //Static Test Mode
   digitalWrite(grnLED, HIGH);
   digitalWrite(bluLED, LOW);
   digitalWrite(redLED, HIGH);

   flightTime();

   if(flight_time >= 1.2e8){ //if threshold time reached
     digitalWrite(grnLED, HIGH);
     digitalWrite(redLED, LOW);
     digitalWrite(bluLED, LOW);
  
     servoX.detach();
     servoY.detach();
     file.close();
     delay(100);

     while(1){
        buzzer.sound(NOTE_D7, 50);
        buzzer.sound(NOTE_D7, 50);
        analogWrite(redLED, random(0,255));
        analogWrite(grnLED, random(0,255));
        analogWrite(bluLED, random(0,255));
     }
   }
 }

errorX = -1 * (Y - desired_angleX); 
errorY = -1 * (Z - desired_angleY);

X_p = kp*errorX;
Y_p = kp*errorY;

X_i = ki * (X_i + errorX * dt);
Y_i = ki * (Y_i + errorY * dt);

X_d = kd*((errorX - previous_errorX)/dt);
Y_d = kd*((errorY - previous_errorY)/dt);

PIDX = X_p + X_i + X_d;
PIDY = Y_p + Y_i + Y_d;

pwmY = ((PIDY * servoY_gear_ratio) + servoX_offset);
pwmX = ((PIDX * servoX_gear_ratio) + servoY_offset);  

previous_errorX = errorX;
previous_errorY = errorY; 

    servoX.write(pwmX); 
    servoY.write(pwmY);
    /*
    Serial.print(flight_time);
    Serial.print("\t");
    Serial.print(micros());
    Serial.print("\t");
    Serial.print("Euler:  ");
    Serial.print(X);
    Serial.print("\t");
    Serial.print(Y);
    Serial.print("\t");
    Serial.print(Z);
    Serial.print("\t");
    Serial.print("Acc:   ");
    Serial.print(ori_acc_prime[1]);
    Serial.print("\t");
    Serial.print(ori_acc_prime[2]);
    Serial.print("\t");
    Serial.print(ori_acc_prime[3]);
    Serial.print("\t");
    Serial.print("PWM:   ");
    Serial.print(pwmX);
    Serial.print("\t");
    Serial.print(pwmY);
    Serial.print("\t");
    Serial.print(systemState);
    Serial.print("\n");
    */

    file.println("0");
    file.print(",");
    file.print(systemState);
    file.print(",");
    file.print(micros());
    file.print(",");
    file.print(flight_time);
    file.print(",");
    file.print(temp);
    file.print(",");
    file.print(pressure, 3);
    file.print(",");
    file.print(altitude, 3);
    file.print(",");
    file.print(rax, 5);
    file.print(",");
    file.print(ray, 5);
    file.print(",");
    file.print(raz, 5);
    file.print(",");
    file.print(ax, 5);
    file.print(",");
    file.print(ay, 5);
    file.print(",");
    file.print(az, 5);
    file.print(",");
    file.print(rgx, 5);
    file.print(",");
    file.print(rgy, 5);
    file.print(",");
    file.print(rgz, 5);
    file.print(",");
    file.print(gx, 5);
    file.print(",");
    file.print(gy, 5);
    file.print(",");
    file.print(gz, 5);
    file.print(",");
    file.print(X, 5);
    file.print(",");
    file.print(Y, 5);
    file.print(",");
    file.print(Z, 5);
    file.print(",");
    file.print(X_p, 5);
    file.print(",");
    file.print(X_i, 5);
    file.print(",");
    file.print(X_d, 5);
    file.print(",");
    file.print(Y_p, 5);
    file.print(",");
    file.print(Y_i, 5);
    file.print(",");
    file.print(Y_d, 5);
    file.print(",");
    file.print(errorX, 5);
    file.print(",");
    file.print(errorY, 5);
    file.print(",");
    file.print(pwmX, 5);
    file.print(",");
    file.print(pwmY, 5);
    file.print(",");
    file.print(PIDX, 5);
    file.print(",");
    file.print(PIDY, 5);
    file.print(",");
}


