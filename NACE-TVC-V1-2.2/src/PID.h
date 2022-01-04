/*
PID controller. This is the control algorithm that outputs the angle needed by the TVC mount to correct the error. 
*/
#ifndef PID_h
#define PID_H

#include <Navigation.h>
#include <Settings.h>
#include <Outputs.h>

//"PID" Constants - These are always changing
float X_p = 0;
float Y_p = 0;
float Y_i = 0;
float X_i = 0;
float X_d = 0;
float Y_d = 0;

float previous_errorX, previous_errorY; //Previous errors - used for the integral

float PIDX, PIDY; //pid outputs

void convert_to_vehicle(){ //converting pid angles so that it accounts for roll: pitch and yaw are decoupled from roll using this function 
    float cs = cos(data.X * DEG_TO_RAD);
    float sn = sin(data.X * DEG_TO_RAD);
 
    data.PIDY = (PIDX * sn) + (PIDY * cs);
    data.PIDX = (PIDX * cs) - (PIDY * sn);
}

void runPID() { //two variations for each axis
    //finding the error
    data.errorX = -1 * (data.Y - Setpoint); 
    data.errorY = -1 * (data.Z - Setpoint);
    
    //proportional gain
    X_p = kp * data.errorX;
    Y_p = kp * data.errorY;
    
    //Integral gain
    X_i = ki * (X_i + data.errorX * dt);
    Y_i = ki * (Y_i + data.errorY * dt);
    
    //Derivative gain
    X_d = kd * ((data.errorX - previous_errorX) / dt);
    Y_d = kd * ((data.errorY - previous_errorY) / dt);
    
    //adding all the constants up
    PIDX = X_p + X_i + X_d;
    PIDY = Y_p + Y_i + Y_d;
    
    //adding the offsets and ratios to the output
    float pwmY = ((data.PIDY * servoY_gear_ratio) + servoY_offset);
    float pwmX = ((data.PIDX * servoX_gear_ratio) + servoX_offset);  

    //writing the data to the servos!
    ServoXYwrite(pwmX, pwmY);

    previous_errorX = data.errorX;
    previous_errorY = data.errorY;
}

#endif