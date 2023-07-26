#pragma once

#include <Arduino.h>
#include "ICM_20948.h"
#include "WorkingDir.h"
#include "Communication.h"
#include <MadgwickAHRS.h>

#define WIRE_PORT Wire // Your desired Wire port. 
#define AD0_VAL 0x69

/**
 * Class for talking to the IMU
*/
class IMU
{
public:
    IMU();
    void Update();
    bool dataReady();
    Point getAccel();
    Point getVelocity(Point accel, int dt);
    double getRotation(int dt);
    void plotData(double orient, double vel, double accel);
    void printScaledAGMT();
    void printPaddedInt16b(int16_t val);
    void printRawAGMT(ICM_20948_AGMT_t agmt);
    void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);

private:
    ICM_20948_I2C myICM; // Create an ICM_20948_I2C object
    Point velocity;
    double rotation;

    Point accelRaw;
    double gyroRaw;

    Madgwick filter; // Create a Madgwick filter object
};
