/*
These functions are used to read and filter live sensor data and manipulate the data
Navigation also contains state exit conditions that move the states machine forward during flight
Also includes abort features and error checking feature
Contains two open source libraries used to comm. with the onboard sensors: .pio/libdeps
*/
#ifndef Navigation_h
#define Navigation_h

#include <Arduino.h>
#include "BMI088.h" //in .pio/libdeps
#include <math.h> //built in math functions
#include <Filtering.h>
#include <BMP388_DEV.h> //in .pio/libdeps
#include <Outputs.h>

unsigned long fire_time;
double cumSum;
double dt, previous_time;

unsigned long flight_starttime, flight_time; //used to calculate time elapsed after detected lift-off

struct Quaternion { //structure to hold Quaternion
 float w, x, y, z;
};
//Orientation Quaternions - A and B for computing Orientation (1,0,0,0) is the initial quaternion
Quaternion A = {1, 0, 0, 0}; 
Quaternion B = {1, 0, 0, 0}; 
Quaternion Orientation = {1, 0, 0, 0}; 

struct Data { //all the data that we log
  uint8_t systemState;
  unsigned long flight_time;
  float rpressure, raltitude, rtemp, temp, pressure, altitude;
  float rax, ray, raz, ax, ay, az;
  float wfXa, wfYa, wfZa, Vx, Vy, Vz;
  float px, py, pz;
  float rgx, rgy, rgz, gx, gy, gz;
  float X, Y, Z;
  float errorX, errorY, PIDX, PIDY;
};

Data data; //initializing structure

float flightAltitude, groundAltitude; //used for detecting apogee of the rocket's flight

//used to periodically check if rocket has reached max altitude
float bmpIntervalPrevious, bmpIntervalCurrent, intervalAltitude; 
float apogeeIntervalCurrent, apogeeIntervalPrevious;

BMP388_DEV bmp388; //creating BMP388_DEV class: in .pio/libdeps/BMP388_DEV

//Initializing IMU: See BOSCH BMI088 Datasheet for more details
Bmi088Accel accel(Wire,0x18); //Initialize BMI088 accelerometer:I2C mode at address 0x18
Bmi088Gyro gyro(Wire,0x68); //Initrialize BMI088 gyuroscope:I2C mode at address 0x68

uint8_t FTstCollected = 0; //determines if the start time is collected to start timing the data when the rocket lifts off

//////////////////////////////////
//////GNC Functions/////
void getScalar(float theta) { //get quaternion scalar for computing quaternion orientation
  B.w = cos(theta / 2);
}
void getVector(float x, float y, float z, float quatNorm, float theta) { //get quaternion vector for computing quaternion orientation
  B.x = (x / quatNorm) * sin(theta / 2);
  B.y = (y / quatNorm) * sin(theta / 2);
  B.z = (z / quatNorm) * sin(theta / 2);
}

 Quaternion hamiltonProduct(Quaternion A, Quaternion B) { //function that gets the Hamilton Product of two quats. used for converting ref. frames and calculating orientation quat
  Quaternion product;
 
  product.w = (A.w * B.w) - (A.x * B.x) - (A.y * B.y) - (A.z * B.z);
  product.x = (A.w * B.x) + (A.x * B.w) + (A.y * B.z) - (A.z * B.y);
  product.y = (A.w * B.y) - (A.x * B.z) + (A.y * B.w) + (A.z * B.x);
  product.z = (A.w * B.z) + (A.x * B.y) - (A.y * B.x) + (A.z * B.w);
 
  return product;
}

 float sinxcosx(float a, float b, float c, float d){ // function that is used to get values from the quaternion and calculating the Euler Angles
     return 2 * (a * d + b * c);
 }
 
 float cosxcosp(float c, float d){ //another function that is used to get values from the quaternion and to calculate the Euler Angles
     return 1 - 2 * (c * c + d * d);
 }

 void FlightTime(){ //time keeper function that starts timing once liftoff is detected
  if(systemState >= 2) {
    if(FTstCollected == 0){
      flight_starttime = micros();
      FTstCollected++;
      } else{
        flight_time = micros() - flight_starttime;
        }
    }
 }

 void Quat_to_Euler(){ //converts the orientation quaternion to nice and easy euler angles
 float sinr_cosp = 2 * (Orientation.w * Orientation.x + Orientation.y * Orientation.z);
 float cosr_cosp = 1 - 2 * (Orientation.x * Orientation.x + Orientation.y * Orientation.y);
     data.X = (atan2(sinr_cosp, cosr_cosp)) * RAD_TO_DEG;
 
     
 float sinp = 2 * (Orientation.w * Orientation.y - Orientation.z * Orientation.x);
    if (abs(sinp) >= 1)
        data.Y = (copysignf(HALF_PI, sinp)) * RAD_TO_DEG;
    else
        data.Y = (asin(sinp)) * RAD_TO_DEG;
 
 float siny_cosp = 2 * (Orientation.w * Orientation.z + Orientation.x * Orientation.y);
 float cosy_cosp = 1 - 2 * (Orientation.y * Orientation.y + Orientation.z * Orientation.z);
    data.Z = (atan2(siny_cosp, cosy_cosp)) * RAD_TO_DEG;
 }
  
 void calculateQuaternion(){
   //Quatnorm and theta calculated are calcualted from filtered gyro and delta time, and run through above functions to calculate Quat. orientation
   float quatNorm = sqrt(sq(data.gx) + sq(data.gy) + sq(data.gz));
   quatNorm = max(abs(quatNorm), 1e-12f);
 
   float theta = quatNorm * dt;
 
   getScalar(theta); //calculating Quat B.w using theta
   getVector(data.gx, data.gy, data.gz, quatNorm, theta); //calculating the rest of Quat B
 
   Quaternion Orientation = hamiltonProduct(A, B); //we get the hamilton product of Quat A (prev Orientation) and B and it gives us current orientation

   //setting current quat. to prev quat for future calculation
   A.w = Orientation.w;
   A.x = Orientation.x;
   A.y = Orientation.y;
   A.z = Orientation.z;
 }

 void convertAccelFrame(){
   Quaternion accReadings = {0, data.ax, data.ay, data.az}; //asigning filtered accelerometer values to the quaternion structure
 
   //Now that the orientation Quaternion has been calculated from the ang. rates from the gyro, we can use this orientation quat to convert reference frames of the acceleration
   //The provided acceleration from the accelerometer is in vehicle reference frame, which cannot be put into the states machine
   //So we convert the accelerometer frame of reference to world frame, which is possible because we have the Quaternion orientation
   //We also convert to world frame so we can subtract gravity to integrate/double integrate to get relative vel. and pos. 
   Quaternion wF_acc = hamiltonProduct(Orientation, accReadings); //finding the first hamilton product of the orientation and the acc. readings to convert reference frame
 
   //second hamilton product of the acc. readings: We don't stick this into the functioin for 2 reasons
   //1: we only need x,y,z of the product
   //2: we need to also multiply by -1 to flip the ref. frame to the correct orientation
   data.wfXa = (wF_acc.w * Orientation.x * -1 + wF_acc.x * Orientation.w + wF_acc.y * Orientation.z * -1 - wF_acc.z * Orientation.y * -1) - Gravity;
   data.wfYa = wF_acc.w * Orientation.y * -1 - wF_acc.x * Orientation.z * -1 + wF_acc.y * Orientation.w + wF_acc.z * Orientation.x * -1;
   data.wfZa = wF_acc.w * Orientation.z * -1 + wF_acc.x * Orientation.y * -1 - wF_acc.y * Orientation.x * -1 + wF_acc.z * Orientation.w;
 }

 void calculateVelocity(){ 
   //Calculating velocity in XYZ by integrating WF acc values
   data.Vx = data.Vx + (data.wfXa * dt);
   data.Vy = data.Vy + (data.wfYa * dt);
   data.Vz = data.Vz + (data.wfZa * dt);

 }

 void calculatePostition(){
   //Calculating position in XYZ by integrating Vel. 
   data.px = data.px + (data.Vx * dt);
   data.py = data.py + (data.Vy * dt);
   data.pz = data.pz + (data.Vz * dt);
 }

 void readSensors(){ //collects, filters, manipulates sensor data for orientation, wf acc, and barometeric pressure data

   /////Reading and converting IMU Data///////////
   accel.readSensor();
   gyro.readSensor();
   //these functions initialize I2C comms between MCU & Sensor to collect data from the IMU: Functions can be found in .pio/libdeps/Bolder Flight Systems BMI088
   data.rax = accel.getAccelX_mss();
   data.ray = accel.getAccelY_mss();
   data.raz = accel.getAccelZ_mss();
   data.rgx = gyro.getGyroX_rads();
   data.rgy = gyro.getGyroY_rads();
   data.rgz = gyro.getGyroZ_rads();
 
   //Running raw values from acc. through the Kalman Filter from <Filtering.h> and subtracting biases calculatied at the beginning 
   data.ax = KalmanFilterX.updateEstimate(data.rax) - axBias;
   data.ay = KalmanFilterY.updateEstimate(data.ray) - ayBias;
   data.az = KalmanFilterZ.updateEstimate(data.raz) - azBias;
 
   //No kalman filter for gyro since it is not subject as much to vibrations, and drift rate is very low and insignificant for a ~30 sec flight
   data.gx = data.rgx - gxBias;
   data.gy = data.rgy - gyBias;
   data.gz = data.rgz - gzBias;
 
   calculateQuaternion(); //converts angular rates and delta time to quaternion orientation
   convertAccelFrame(); //converting the acceleration to world frame
   //Now we convert the Orientation quaternion into Euler angles (x,y,z) to use in our PID and datalogging 
   Quat_to_Euler();

   ////////Reading and Filtering Barometer data
   bmp388.startForcedConversion(); //starting forced mode of the BMP388 barometer (details can be found in the datasheet)
   bmp388.getTempPres(data.rtemp, data.rpressure); //this function assigns raw measurements from the BMP388 to the members of the Data structure
   //running raw baro. data through the Kalman Filter
   data.pressure = KalmanFilterP.updateEstimate(data.rpressure);
   data.temp = KalmanFilterT.updateEstimate(data.rtemp);
   data.altitude = ((float)powf(1013.23f / data.pressure, 0.190223f) - 1.0f) * (data.temp + 273.15f) / 0.0065f; //converts temp and pressure to a altitude

   //calculating velocity by integrating wf acc. and calculating position by integrating vel.
   calculateVelocity();
   calculatePostition();
}

void calculateBias(){ //reads 100 values from the acc. and gyro. and averages them (while still) to calculate biases caused by temp, e. noise, etc. 
  uint8_t counter = 0;
  float gxIntegration = 0;
  float gyIntegration = 0;
  float gzIntegration = 0;

  float axIntegration = 0;
  float ayIntegration = 0;
  float azIntegration = 0;
 
  for(int i = 0; i < 104; i++){
    readSensors();
    if(counter >= 4){ //this filters out the first 4 readings because these can be unstable
    gxIntegration += data.gx;
    gyIntegration += data.gy;
    gzIntegration += data.gz;

    axIntegration += data.ax - Gravity; //substract gravity 
    ayIntegration += data.ay;
    azIntegration += data.az;
    
    }
    counter++;
  }
  gxBias = gxIntegration / 100; //these values are later subtracted from measurements to reduce gyro. drift
  gyBias = gyIntegration / 100;
  gzBias = gzIntegration / 100; 

  axBias = axIntegration / 100; //these values are later subtracted from measurements to reduce acc. drift
  ayBias = ayIntegration / 100;
  azBias = azIntegration / 100;
}

void initializeSensors(){
  //Initializing and Checking for errors within the Bosch BMI088 IMU
  uint8_t status;
  status = accel.begin();
  status = accel.setOdr(Bmi088Accel::ODR_400HZ_BW_40HZ); 
  status = accel.setRange(Bmi088Accel::RANGE_12G); //should not pass this threshold in flight

  if(status < 0){
    Serial.println(F("Accel. Initialization Error"));
    indicateError();
  }

  status = gyro.begin();
  status = gyro.setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
  status = gyro.setRange(Bmi088Gyro::RANGE_2000DPS); //2000 degrees per second threshold

  if(status < 0){
    Serial.println(F("Gyro Initialization Error"));
    indicateError();
  }

  //Initializing and Checking for errors within the Bosch BMP388 Barometer
  bmp388.begin(BMP388_I2C_ALT_ADDR); //0x76 See datasheet for I2C address details
  bmp388.setPresOversampling(OVERSAMPLING_X8); //setting oversampling for barometer: gets 8 of the readings and averages them 
  bmp388.setTempOversampling(OVERSAMPLING_X2); //setting oversampling for the thermometer: gets 2 of the readings and averages them
  bmp388.setIIRFilter(IIR_FILTER_16);

  //Checking for small amounts of bias within the Barometer and Gyroscope while vehicle is still to account for temp and other noise
  calculateBias(); //(<Filtering.h>)
}

////////State Exit Conditions////////////
//this function checks to see if we are getting constant zeros from the IMU
//I have noticed with repeated use, the IMU sometimes just outputs straight zeros, so this checks that this doesn't happen
void errorCheckIMU(){ 
  for(uint8_t i; i <= 500; i++){
    //we use PROGMEM to use less SRAM and add the variables to FLASH: we can use this because it is in setup so it won't slow down actual loop...
    double gx[] PROGMEM = {gyro.getGyroX_rads()};
    double gy[] PROGMEM = {gyro.getGyroY_rads()};
    double gz[] PROGMEM = {gyro.getGyroZ_rads()};

    //adding it all up just to be triple sure that the error is there (0.0 + 0.0 + 0.0 = 0.0 anyways)
    cumSum = {pgm_read_float_near(gx) + pgm_read_float_near(gy) + pgm_read_float_near(gz)};
    
    if(cumSum == 0.0f){
      indicateError();
      Serial.printf("IMU Output Error: Power Cycle NACE");
    } 
  } 

  
}

void detectNavReady(){ //velocity should be relatively low: def won't be zero from drift...
  if(data.Vx >= velocityTerminationThreshold){ //velocityTerminationThreshold set up in settings.h
    indicateError();
  }
   else if(data.Vy >= velocityTerminationThreshold){
    indicateError();
  }
   else if(data.Vz >= velocityTerminationThreshold){
    indicateError();
  } else{
   systemState++;
  }
}

void detectLaunch(){ //self explanatory :)
  if(data.wfXa >= 2){ //if world frame acc. is greater than 2m/s/s a launch is detected
    systemState++;
  }
}
void abortFlight(){ //sets pyro pin high which triggers load switch, which fires pyro
  digitalWrite(pyro, HIGH);
  detachServos();
  systemState = 6;
}
void Flight_Termination_Handler(){ //checks to see if the rocket's orientation is past a certain threshold: must turn enableFTS on to enable abort functionality
  if(enableFTS == true){
     if(data.Y >= 30 || data.Y <= -30){
       abortFlight();
      }
  
 
      else if(data.Z >= 30 || data.Z <= -30){
       abortFlight();
     }
 } else{ 
   //do nothing 
 }
}
 void detectBurnout(){ //self explanatory :)
   if(data.wfXa <= 2){
     systemState++;
   }
 }

 void detectApogee(){ //periodically check if the rocket is at the top of its flight: periodically because we dont want this function triggered by noise or drift
   bmpIntervalCurrent = micros();
   apogeeIntervalCurrent = micros();
    if(bmpIntervalCurrent - bmpIntervalPrevious >= bmpInterval) { //periodically updates interval altitude
      bmpIntervalPrevious = bmpIntervalCurrent;
      intervalAltitude = data.altitude;
 }
  if(apogeeIntervalCurrent - apogeeIntervalPrevious >= apogeeCheck && intervalAltitude > data.altitude + 0.5f){
     firePyro();
     fire_time = micros();
     systemState++;
    }
 }

 void detectLanding(){ //self explanatory :)
   //if(data.rgx <= 5 && data.rgy <= 5 && data.rgz <= 5 && systemState == 4){ //gyros should be relatively low upon landing (threshold is 1 rad/sec)
    // systemState++;                                                         //we also double check systemstate just in case the second else if triggers the exit condition
   //} 
   if(micros() >= fire_time + 2e7f &&  systemState == 4){
     systemState++;
   }
 }


#endif