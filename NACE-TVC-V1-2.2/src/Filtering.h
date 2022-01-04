#ifndef Filering_h
#define Filtering_h

#include <Arduino.h>
#include <math.h>
#include <Navigation.h>

//small bias values that are calculate in setup. these values change on many variables
float gxBias, gyBias, gzBias; 
float axBias, ayBias, azBias;

class KalmanFilter
{
public:
    KalmanFilter(float meUn, float errUn, float procVariance);
    double updateEstimate(double measurment);
    float getMeasurementUncertainty();
    float getErrorUncertainty();
    float getProcessVariance();

private:
    float meUn;
    float errUn;
    double procVariance;
    double estimate;
    double prevEstimate;
    double K;
};
KalmanFilter::KalmanFilter(float meUn, float errUn, float procVariance)
{
    this->meUn = meUn;
    this->errUn = errUn;
    this->procVariance = procVariance;
}

double KalmanFilter::updateEstimate(double measurement)
{
    K = errUn / (errUn + meUn);
    estimate = prevEstimate + K * (measurement - prevEstimate);
    errUn = fabs(prevEstimate - estimate) + (1.0f - K) * errUn;
    prevEstimate = estimate;

    return estimate;
}

float KalmanFilter::getMeasurementUncertainty() {
    return meUn;
}

float KalmanFilter::getErrorUncertainty() {
    return errUn;
}

float KalmanFilter::getProcessVariance() {
    return procVariance;
}

//Kalman Filters for Accelerometer and Barometer Readings to increase robustness of readings
//Setting Measurement Uncertainties and Proc. Variance to tune Acc. and Barometer
KalmanFilter KalmanFilterX(1,.1,1);
KalmanFilter KalmanFilterY(0.5,0.1,0.1);
KalmanFilter KalmanFilterZ(0.5,0.1,0.1);
 
KalmanFilter KalmanFilterP(0.5,0.1,0.01);
KalmanFilter KalmanFilterT(0.5,0.1,0.1);
#endif