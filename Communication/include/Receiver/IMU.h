#pragma once

#include <Arduino.h>
#include "ICM_20948.h"
#include "Communication.h"
#include "Hardware.h"

#define WIRE_PORT Wire // Your desired Wire port. 

#define TO_RAD 0.01745329251

// on boot, avoid sending sync messages for this long
// if a sync message receive within this interval, use it at 100% weight
#define BOOT_GYRO_MERGE_MS 1000

/**
 * Class for talking to the IMU
*/
class IMU
{
public:
    IMU();
    void Initialize(enum board_placement placement);
    void Update();
    void MergeExternalInput(board_placement placement, float rotation, float velocity);
    void ForceCalibrate();
    bool dataReady();
    Point getAccel();
    Point getVelocity();
    double getRotation();
    double getRotationVelocity();
    void plotData(double orient, double vel, double accel);
    void printScaledAGMT();
    void printPaddedInt16b(int16_t val);
    void printRawAGMT(ICM_20948_AGMT_t agmt);
    void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);

    bool isImuHealthy();

    ICM_20948_I2C myICM; // Create an ICM_20948_I2C object
private:
    void _updateGyro(double deltaTimeMS);
    void _updateAccelerometer(double deltaTimeMS);

    Point _velocity;
    Point _calibrationAccel;
    Point _currAcceleration;
    Point _prevAcceleration;
    double _rotation = 0;
    double _currRotVelZ = 0;

    // the following variables are used to calibrate the gyro
    double _calibrationRotVelZ = 0;
    double _prevRotVelZ = 0;

    unsigned long _lastAccelerateTimeMS = 0;

    double _prevTimeMS = 0;

    enum board_placement _placement = invalidPlacement;

    double _gyro_scale_factor = 0;
    float (ICM_20948::*_gyro_axis)(void) = nullptr;
    float _smoothRotationVelocity = 0;

    float all_velocities[4] = {0.0f};
    uint32_t lastPacketTimestamp[4] = {0};
    uint32_t stoppedMovingTimestamp[4] = {0};
    uint32_t startedMovingTimestamp = 0;

    bool _imu_healthy = true;

};
