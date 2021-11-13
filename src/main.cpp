#include <Arduino.h>
#include "BMI088.h"   
#include <math.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>
#include <BMP388_DEV.h>
#include <Buzzer.h>
#include <SD.h>


struct Quaternion {
 float w, x, y, z;
};

Quaternion A = {1, 0, 0, 0};
Quaternion B = {1, 0, 0, 0};
Quaternion Orientation = {1, 0, 0, 0};

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
float raltitude, rpressure;

SimpleKalmanFilter KalmanFilterX(2.123, 2, 1);
SimpleKalmanFilter KalmanFilterY(1.98, 2, 1);
SimpleKalmanFilter KalmanFilterZ(1.98, 2, 1);

SimpleKalmanFilter KalmanFilterP(2, 2, 1);
SimpleKalmanFilter KalmanFilterA(2, 2, 1);

Servo servoX;
Servo servoY;

float wfXa, wfYa, wfZa;
float X, Y, Z;
float Vx, Vy, Vz;

float PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY;

unsigned long counter;
//PID Gains//
float kp = .3;
float ki = .3;
float kd = 0.05;

//"PID" Constants
float X_p = 0;
float Y_p = 0;
float Y_i = 0;
float X_i = 0;
float X_d = 0;
float Y_d = 0;

bool stateMachine_enable = true;
bool abortFunctionality_enable = true;

Bmi088Accel accel(Wire,0x18);
Bmi088Gyro gyro(Wire,0x68);

float previous_time, dt;
float abort_time, flight_time, flight_starttime;
uint8_t FTstCollected = 0;

float bmpIntervalPrevious, bmpIntervalCurrent, intervalAltitude;
float apogeeIntervalCurrent, apogeeIntervalPrevious;
float pyroTime;

float gxBias, gyBias, gzBias;

int systemState = 0;
bool abort_armed = false;

//Upright Angle of the Gyroscope
const uint8_t desired_angleX = 0;//servoY
const uint8_t desired_angleY = 0;//servoX
//Offsets for tuning 
const float servoY_offset = 47;
const float servoX_offset = 126.5;
//Position of servos through the startup function
int servoXstart = servoY_offset * -1;
int servoYstart = servoX_offset;
//Ratio between servo gear and tvc mount
const uint8_t servoX_gear_ratio = 2;
const uint8_t servoY_gear_ratio = 2;

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

  for(int i = 0; i < 100; i++){
    gyro.readSensor();

    float gx = gyro.getGyroX_rads();
    float gy = gyro.getGyroY_rads();
    float gz = gyro.getGyroZ_rads();

    gxIntegration = gxIntegration + gx;
    gyIntegration = gyIntegration + gy;
    gzIntegration = gzIntegration + gz;
  }
  gxBias = gxIntegration / 100;
  gyBias = gyIntegration / 100;
  gzBias = gzIntegration / 100;
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
  
  file.print("void,state,Time,fliTime,rpressure,raltitude,Temp,Pres,Alti,rax,ray,raz,fax,fay,faz,wfXa,wfYa,wfZa,Vx,Vy,Vz,rgx,rgy,rgz,fgx,fgy,fgz,X,Y,Z,ErrorX,ErrorY,PIDX,PIDY");
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

  file = SD.open(fileNamedata, O_WRITE | O_CREAT);
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
  B.w = cos(theta / 2);
}

void getVector(float x, float y, float z, float quatNorm, float theta) {
  B.x = (x / quatNorm) * sin(theta / 2);
  B.y = (y / quatNorm) * sin(theta / 2);
  B.z = (z / quatNorm) * sin(theta / 2);
}

void abortFlight(){
  systemState = 6;
  digitalWrite(pyro, HIGH);
  servoX.detach();
  servoY.detach();
  file.close();

  digitalWrite(bluLED, LOW);
  digitalWrite(grnLED, LOW);
  digitalWrite(redLED, HIGH);
  while(1){
  }
}

int checkOri(float Y, float Z){ //returns 0 if no anomoly, returns 1 to abort
 uint8_t result = 0;
 if(Y >= 30 || Y <= -30){
   if(abort_armed == true){
     result = 1;
   }
 }

 if(Z >= 30 || Z <= -30){
   if(abort_armed == true){
     result = 1;
   }
 }
 return result;
}

void flightTime() {
 if(systemState >= 2) {
  if(FTstCollected == 0){
    flight_starttime = micros();
    FTstCollected++;
      }
    
    else{
        flight_time = micros() - flight_starttime;
      }

  }
}

 Quaternion hamiltonProduct(Quaternion A, Quaternion B) {
  Quaternion product;

  product.w = (A.w * B.w) - (A.x * B.x) - (A.y * B.y) - (A.z * B.z);
  product.x = (A.w * B.x) + (A.x * B.w) + (A.y * B.z) - (A.z * B.y);
  product.y = (A.w * B.y) - (A.x * B.z) + (A.y * B.w) + (A.z * B.x);
  product.z = (A.w * B.z) + (A.x * B.y) - (A.y * B.x) + (A.z * B.w);

  return product;
}

float returnVelocity(float acc, float dt) {
  float linVelocity;
  linVelocity = acc * dt;
  return linVelocity;
}

void loop()
{

 dt = (micros() - previous_time) / 1000000;
 previous_time = micros();
 
 flightTime();
 
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

 float gx = rgx - gxBias;
 float gy = rgy - gyBias;
 float gz = rgz - gzBias;

 float quatNorm = sqrt(sq(gx) + sq(gy) + sq(gz));
 quatNorm = max(abs(quatNorm), 1e-12);

 float theta = quatNorm * dt;

 getScalar(theta);
 getVector(gx, gy, gz, quatNorm, theta);

 Quaternion Orientation = hamiltonProduct(A, B);
 A.w = Orientation.w;
 A.x = Orientation.x;
 A.y = Orientation.y;
 A.z = Orientation.z;

 Quaternion accReadings = {0, ax, ay, az};

 Quaternion wF_acc = hamiltonProduct(Orientation, accReadings);

 wfXa = (wF_acc.w * Orientation.x * -1 + wF_acc.x * Orientation.w + wF_acc.y * Orientation.z * -1 - wF_acc.z * Orientation.y * -1) - 9.81;
 wfYa = wF_acc.w * Orientation.y * -1 - wF_acc.x * Orientation.z * -1 + wF_acc.y * Orientation.w + wF_acc.z * Orientation.x * -1;
 wfZa = wF_acc.w * Orientation.z * -1 + wF_acc.x * Orientation.y * -1 - wF_acc.y * Orientation.x * -1 + wF_acc.z * Orientation.w;

 Vx = returnVelocity(wfXa, dt);
 Vy = returnVelocity(wfYa, dt);
 Vz = returnVelocity(wfZa, dt);

 float sinr_cosp = 2 * (Orientation.w * Orientation.x + Orientation.y * Orientation.z);
 float cosr_cosp = 1 - 2 * (Orientation.x * Orientation.x + Orientation.y * Orientation.y);
     X = (atan2(sinr_cosp, cosr_cosp)) * RAD_TO_DEG;
  
     
 float sinp = 2 * (Orientation.w * Orientation.y - Orientation.z * Orientation.x);
    if (abs(sinp) >= 1)
        Y = (copysignf(HALF_PI, sinp)) * RAD_TO_DEG;
    else
        Y = (asin(sinp)) * RAD_TO_DEG;

 float siny_cosp = 2 * (Orientation.w * Orientation.z + Orientation.x * Orientation.y);
 float cosy_cosp = 1 - 2 * (Orientation.y * Orientation.y + Orientation.z * Orientation.z);
    Z = (atan2(siny_cosp, cosy_cosp)) * RAD_TO_DEG;

 bmp388.startForcedConversion();
 bmp388.getMeasurements(temp, rpressure, raltitude);

 altitude = KalmanFilterA.updateEstimate(raltitude);
 pressure = KalmanFilterP.updateEstimate(rpressure);

 bmpIntervalCurrent = micros();
 apogeeIntervalCurrent = micros();

 const float bmpInterval = 250000;
 const float apogeeCheck = 250000;

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
    if(systemState == 1 && wfXa >= 2){
      systemState++; //move system on to powered flight mode
      abort_armed = !abort_armed;
      digitalWrite(bluLED, HIGH);
      digitalWrite(redLED, HIGH);
      digitalWrite(grnLED, HIGH);
        }
 
 
    if(systemState == 2 && wfXa <= 2){
      systemState++; //move system on to unpowered coast
      digitalWrite(bluLED, HIGH);
      digitalWrite(redLED, LOW);
      digitalWrite(grnLED, LOW);
        }

    if(apogeeIntervalCurrent - apogeeIntervalPrevious >= apogeeCheck && systemState == 3){
      if(intervalAltitude > altitude + 0.5){
        systemState++; //move system to descent
        digitalWrite(bluLED, LOW);
        digitalWrite(grnLED, HIGH);
        digitalWrite(pyro, HIGH);
        pyroTime = micros();
      }
    }

 
    if(micros() >= pyroTime + 20000000  && systemState == 4){
        systemState++; //move system on to landed
        digitalWrite(bluLED, LOW);
        digitalWrite(redLED, HIGH);
        digitalWrite(grnLED, HIGH);
        digitalWrite(pyro, LOW);

        buzzer.sound(NOTE_D7, 50);
        buzzer.sound(NOTE_D5, 50);

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
 /*else{ //Static Test Mode
   digitalWrite(grnLED, HIGH);
   digitalWrite(bluLED, LOW);
   digitalWrite(redLED, HIGH);

   flightTime();

   if(flight_time >= 1.2e7){ //if threshold time reached
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
*/
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

float cs = cos(X * DEG_TO_RAD);
float sn = sin(X * DEG_TO_RAD);

float trueZOut = (PIDX * sn) + (PIDY * cs);
float trueYOut = (PIDX * cs) - (PIDY * sn); 

pwmY = ((trueZOut * servoY_gear_ratio) + servoY_offset);
pwmX = ((trueYOut * servoX_gear_ratio) + servoX_offset);  

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
    Serial.print(wfXa);
    Serial.print("\t");
    Serial.print(wfYa);
    Serial.print("\t");
    Serial.print(wfZa);
    Serial.print("\t");
    Serial.print("Velocity:  ");
    Serial.print(Vx);
    Serial.print("\t");
    Serial.print(Vy);
    Serial.print("\t");
    Serial.print(Vz);
    Serial.print("\t");
    Serial.print("PWM:   ");
    Serial.print(pwmX);
    Serial.print("\t");
    Serial.print(pwmY);
    Serial.print("\t");
    Serial.print(systemState);
    Serial.print("\n");
    */
    file.print("\n");
    file.print(F(","));
    file.print(systemState);
    file.print(F(","));
    file.print(micros());
    file.print(F(","));
    file.print(flight_time);
    file.print(F(","));
    file.print(rpressure, 3);
    file.print(F(","));
    file.print(raltitude, 3);
    file.print(F(","));
    file.print(temp);
    file.print(F(","));
    file.print(pressure, 3);
    file.print(F(","));
    file.print(altitude, 3);
    file.print(F(","));
    file.print(rax, 3);
    file.print(F(","));
    file.print(ray, 3);
    file.print(F(","));
    file.print(raz, 3);
    file.print(F(","));
    file.print(ax, 3);
    file.print(F(","));
    file.print(ay, 3);
    file.print(F(","));
    file.print(az, 3);
    file.print(F(","));
    file.print(wfXa, 3);
    file.print(F(","));
    file.print(wfYa, 3);
    file.print(F(","));
    file.print(wfZa, 3);
    file.print(F(","));
    file.print(Vx, 3);
    file.print(F(","));
    file.print(Vy, 3);
    file.print(F(","));
    file.print(Vz, 3);
    file.print(F(","));
    file.print(rgx, 3);
    file.print(F(","));
    file.print(rgy, 3);
    file.print(F(","));
    file.print(rgz, 3);
    file.print(F(","));
    file.print(gx, 3);
    file.print(F(","));
    file.print(gy, 3);
    file.print(F(","));
    file.print(gz, 3);
    file.print(F(","));
    file.print(X, 3);
    file.print(F(","));
    file.print(Y, 3);
    file.print(F(","));
    file.print(Z, 3);
    file.print(F(","));
    file.print(errorX, 3);
    file.print(F(","));
    file.print(errorY, 3);
    file.print(F(","));
    file.print(trueYOut, 3);
    file.print(F(","));
    file.print(trueZOut, 3);
    file.print(F(","));

    if (counter >= 250){
      file.flush();
      counter = 0;
    }

    counter++; 
}