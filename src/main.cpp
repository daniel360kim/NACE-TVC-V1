#include <Arduino.h>
#include "BMI088.h"   
#include <math.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>
#include <BMP388_DEV.h>

BMP388_DEV bmp388;

float temp, altitude, flightAltitude, groundAltitude, pressure;

SimpleKalmanFilter KalmanFilterX(2.123, 2, 0.01);
SimpleKalmanFilter KalmanFilterY(1.98, 2, 0.01);
SimpleKalmanFilter KalmanFilterZ(1.98, 2, 0.01);

SimpleKalmanFilter KalmanFilterGx(0.5, 0.5, 0.1);
SimpleKalmanFilter KalmanFilterGy(0.5, 0.5, 0.1);
SimpleKalmanFilter KalmanFilterGz(0.5, 0.5, 0.1);

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


//Upright Angle of the Gyroscope
int desired_angleX = 0;//servoY
int desired_angleY = 0;//servoX

//Offsets for tuning 
int servoY_offset = 0;
int servoX_offset = 120;

//Position of servos through the startup function
int servoXstart = servoY_offset * -1;
int servoYstart = servoX_offset;

//Ratio between servo gear and tvc mount
float servoX_gear_ratio = 2;
float servoY_gear_ratio = 2;

//"P" Constants
float X_p = 0;
float Y_p = 0;

//"I" Constants
float Y_i = 0;
float X_i = 0;

//"D" Constants
float X_d = 0;
float Y_d = 0;


//PID Gains
double kp = 1.5;
double ki = .15;
double kd = 0.05;

Bmi088Accel accel(Wire,0x18);
Bmi088Gyro gyro(Wire,0x68);

float previous_time, dt;
float abort_time, flight_time, flight_starttime;
float bmpIntervalPrevious, bmpIntervalCurrent, intervalAltitude;
float apogeeIntervalCurrent, apogeeIntervalPrevious;
float pyroTime;
//States
bool launchpad_idle = false;
bool powered_flight = false;
bool unpowered_flight = false;
bool descent = false;
bool abort_state = false;
bool abort_armed = false;
bool landed = false;
bool glcollected = false;
bool ftcollected = false;

const uint8_t redLED = 8;
const uint8_t grnLED = 18;
const uint8_t bluLED = 9;
const u_int8_t pyroCont = A2;
const u_int8_t pyro = 4;

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

  
  Serial.begin(115200);
  servoX.attach(11);
  servoY.attach(13);
  servoX.write(servoXstart);
  servoY.write(servoYstart);
  
  int status;
  status = accel.begin();
  status = accel.setOdr(Bmi088Accel::ODR_400HZ_BW_40HZ);
  status = accel.setRange(Bmi088Accel::RANGE_12G);
  if(status < 0){
    Serial.println("Accel Initialization Error");
    while(1){}
  }
  status = gyro.begin();
  status = gyro.setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
  status = gyro.setRange(Bmi088Gyro::RANGE_2000DPS);
  if(status < 0){
    Serial.println("Gyro Initialization Error");
    while(1){}
  }
  //Initialize BMP388, and get ground level reference
  bmp388.begin(BMP388_I2C_ALT_ADDR);
  bmp388.setPresOversampling(OVERSAMPLING_X8);
  bmp388.setTempOversampling(OVERSAMPLING_X2);
  bmp388.setIIRFilter(IIR_FILTER_16);
  launchpad_idle = !launchpad_idle;
  
}
void getScalar(float theta) {
  b[0] = cos(theta / 2);
}

void getVector(float x, float y, float z, float quatNorm, float theta) {
  b[1] = (x / quatNorm) * sin(theta / 2);
  b[2] = (y / quatNorm) * sin(theta / 2);
  b[3] = (z / quatNorm) * sin(theta / 2);
}

void abort(){
  abort_state = !abort_state;
  digitalWrite(pyro, HIGH);
  servoX.detach();
  servoY.detach();
  //file.close();
}

void loop()
{

 dt = (micros() - previous_time) / 1000000;
 previous_time = micros(); 
 flight_time = micros() - flight_starttime;

 accel.readSensor();
 gyro.readSensor();
 
 float rax = accel.getAccelX_mss();
 float ray = accel.getAccelY_mss();
 float raz = accel.getAccelZ_mss();
 float rgx = gyro.getGyroX_rads();
 float rgy = gyro.getGyroY_rads();
 float rgz = gyro.getGyroZ_rads();

  // calculate the estimated value with Kalman Filter
 float ax = KalmanFilterX.updateEstimate(rax);
 float ay = KalmanFilterY.updateEstimate(ray);
 float az = KalmanFilterZ.updateEstimate(raz);
 float gx = KalmanFilterGx.updateEstimate(rgx);
 float gy = KalmanFilterGy.updateEstimate(rgy);
 float gz = KalmanFilterGz.updateEstimate(rgz);


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
 if(launchpad_idle == true && ori_acc_prime[1] >= 2){
   launchpad_idle = !launchpad_idle;
   powered_flight = !powered_flight;
   abort_armed = !abort_armed;
   digitalWrite(bluLED, HIGH);
   digitalWrite(redLED, HIGH);
   digitalWrite(grnLED, HIGH);
 }
 
 if(powered_flight == true && ori_acc_prime[1] <= 2){
   powered_flight = !powered_flight;
   unpowered_flight = !unpowered_flight;
   digitalWrite(bluLED, HIGH);
   digitalWrite(redLED, LOW);
   digitalWrite(grnLED, LOW);
   
 }

 if(apogeeIntervalCurrent - apogeeIntervalPrevious >= apogeeCheck && unpowered_flight == true){
   if(intervalAltitude > altitude + 0.5){
     unpowered_flight = !unpowered_flight;
     descent = !descent;
    digitalWrite(bluLED, LOW);
    digitalWrite(grnLED, HIGH);
    digitalWrite(pyro, HIGH);
    pyroTime = micros();
   }
 }

 
 if(micros() >= pyroTime + 20000000  && descent == true){
   landed = !landed;
   descent = !descent;
    digitalWrite(9, HIGH);//turn on all three LEDS (white)
    digitalWrite(8, HIGH);
    digitalWrite(18, HIGH);
    digitalWrite(pyro, LOW);
    servoX.detach();
    servoY.detach();
    //file.close();
    delay(100);
    while(1){
      //buzzer soound
      analogWrite(redLED, random(0,255));
      delay(10);
      analogWrite(grnLED, random(0,255));
      delay(10);
      analogWrite(bluLED, random(0,255));
      delay(10);
    }
 }
 
 if(Y >= 30 || Y <= -30){
   if(abort_armed == true){
     abort();
   }
 }

 if(X >= 30 || X <= -30){
   if(abort_armed == true){
     abort();
   }
 }
 


errorX = Y - desired_angleX; 
errorY = Z - desired_angleY;

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
constrain(pwmX,-10,10);
constrain(pwmY, -10,10);
previous_errorX = errorX;
previous_errorY = errorY; 

    servoX.write(pwmX); 
    servoY.write(pwmY);

    Serial.print("Euler:  ");
    Serial.print(X);
    Serial.print("\t");
    Serial.print(Y);
    Serial.print("\t");
    Serial.print(Z);
    Serial.print("\t");
    Serial.print(pwmX);
    Serial.print("\n");
  


 
 


}



