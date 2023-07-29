#pragma once

#include <Arduino.h>
#include "ICM_20948.h"
#include "Communication.h"
#include <MadgwickAHRS.h>

#define WIRE_PORT Wire // Your desired Wire port. 
#define AD0_VAL 0x69

#define TO_RAD 0.01745329251

/**
 * Class for talking to the IMU
*/
class IMU
{
public:
    IMU();
    void Update(double dt);
    bool dataReady();
    Point getAccel();
    Point getVelocity();
    double getRotation();
    void plotData(double orient, double vel, double accel);
    void printScaledAGMT();
    void printPaddedInt16b(int16_t val);
    void printRawAGMT(ICM_20948_AGMT_t agmt);
    void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);

private:
    ICM_20948_I2C myICM; // Create an ICM_20948_I2C object
    Point velocity;
    Point calibrationAccel;
    Point acceleration;
    double rotation;

    // the following variables are used to calibrate the gyro
    double zVelocityCalibration = 0;
    double prevRotVelZ = 0;

    Madgwick filter; // Create a Madgwick filter object
};
