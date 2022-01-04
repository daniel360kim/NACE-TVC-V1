/*
Most of the settings that can be changed before the flight
Some changeable settings are just kept within functions because they are rarely (probably never) changed
*/

#ifndef Settings_h
#define Settings_h

  #define Gravity 9.80665f
  
  ///PID Gains that are tuned 
  const float kp = 0.3;
  const float ki = 0.4;
  const float kd = 0.07;
  
  //Gear ratio between servos and tvc mount
  const int gearRatio = 2;

  //output limits of the servo motors limited by the TVC mount
  const int outputLimitHigh = 10;
  const int outputLimitLow = -10;
  
  //the angle that the rocket is pointing (always zero or straight)
  const uint8_t Setpoint = 0;
  
  //abort is pretty much useless right now so we have it disabled
  bool abort_armed = false;

  //logging rate of the SD card
  unsigned long fastlogPeriodMicros = 500000; //50hz
  unsigned long slowlogPeriodMicros = 1000000; //1hz

  //Offsets for tuning
  const float servoY_offset = 93;
  const float servoX_offset = 99; //plus for counter negative for clockwise
  //Position of servos through the startup function
  int servoXstart = servoY_offset * -1;
  int servoYstart = servoX_offset;
  //Ratio between servo gear and tvc mount
  const float servoX_gear_ratio = 2;
  const float servoY_gear_ratio = 2;

  uint8_t velocityTerminationThreshold = 1; //at what velocity do we stop startup on the launchpad

  //abort settings
  bool enableFTS = false;//enable the flight termination system
  uint8_t abort_threshold = 40; //what angle we call an abort during flight
  float pyOnTimeSec = 0.25; //how long the pyro is on for
 
  //intervals at which the intervals aere checked for apogee detection - so random noise doesn't accidentally trigger system exit function
  const unsigned long bmpInterval = 250000;
  const unsigned long apogeeCheck = 250000;

  //if the gyro threshold is not triggered for a set amount of time, we call a landing anyway as a failsafe
  const unsigned long landingDelayTime = 10000000; //10 seconds

  uint8_t systemState = 0; //state machine variable - just put it here in case we want to change it for some reason during testing...
   
  #endif