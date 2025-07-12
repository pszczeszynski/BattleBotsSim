#include "Receiver/IMU.h"
#include "Hardware.h"

#include <Arduino.h>

#include <cmath>

#define CENTER_GYRO_SCALE_FACTOR -1.0
#define CENTER_GYRO_AXIS gyrZ

#define SIDE_GYRO_SCALE_FACTOR -1.0
#define SIDE_GYRO_AXIS gyrX


#define EXTERNAL_GYRO_MERGE_WEIGHT 0.25
#define PI 3.14159

#define STALE_ANGLE_SYNC_TIMEOUT 150 // consider angle sync up to date if received within last x ms
#define STILL_THRESHOLD_TIMEOUT 500 // consider robot to be still if all IMUs report still for this long


IMU::IMU()
{
    _imu_healthy = true;
}

bool IMU::isImuHealthy()
{
    return _imu_healthy;
}

void IMU::Initialize(enum board_placement placement)
{
    switch(placement)
    {
        case rxWepFront:
        case rxWepRear:
            _gyro_scale_factor = CENTER_GYRO_SCALE_FACTOR;
            _gyro_axis = &ICM_20948::CENTER_GYRO_AXIS;
            break;
        case rxDriveLeft:
        case rxDriveRight:
            _gyro_scale_factor = SIDE_GYRO_SCALE_FACTOR;
            _gyro_axis = &ICM_20948::SIDE_GYRO_AXIS;
            break;
        case tx:
        case invalidPlacement:
        default:
            Serial.println("Error: running IMU init code on tx board ID");
    }
    Serial.println("Initializing IMU...");

    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    // attempt to initialize the imu in a loop until it works
    bool initialized = false;
    while (!initialized)
    {
        // use i2c addres 0x68 (which is false). If true, 0x69 (this was how the the port for the old boards)
        myICM.begin(WIRE_PORT, false);

        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok)
        {
            // fail => print the status
            Serial.println("Trying again...");
            delay(500);
        }
        else
        {
            // success!
            initialized = true;
        }
    }

    // set dps
    ICM_20948_fss_t FSS;
    FSS.a = gpm4;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
    FSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
    ICM_20948_Status_e retval = myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
    if (retval != ICM_20948_Stat_Ok)
    {
        Serial.println("Failed to set max accel for the IMU");
    }

    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors

    myDLPcfg.g = gyr_d119bw5_n154bw3; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                        // gyr_d196bw6_n229bw8
                                        // gyr_d151bw8_n187bw6
                                        // gyr_d119bw5_n154bw3
                                        // gyr_d51bw2_n73bw3
                                        // gyr_d23bw9_n35bw9
                                        // gyr_d11bw6_n17bw8
                                        // gyr_d5bw7_n8bw9
                                        // gyr_d361bw4_n376bw5

    myICM.setDLPFcfg(ICM_20948_Internal_Gyr, myDLPcfg);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("setDLPcfg returned: "));
        Serial.println(myICM.statusString());
    }

    // Choose whether or not to use DLPF
    // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
    ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
    Serial.print(F("Enable DLPF for Gyroscope returned: "));
    Serial.println(myICM.statusString(gyrDLPEnableStat));

    // take an initial reading for the x velocity calibration
    _calibrationRotVelZ = 0;


    // zero out
    _velocity = Point{0, 0, 0};
    _calibrationAccel = Point{0, 0, 0};
    _currAcceleration = Point{0, 0, 0};
    _prevAcceleration = Point{0, 0, 0};
    _rotation = 0;
    _currRotVelZ = 0;
    ForceCalibrate();

    Serial.println("Success!");
}

void IMU::ForceCalibrate()
{
    delay(30);
    double sample1 = -(myICM.*_gyro_axis)() * TO_RAD;
    delay(30);
    double sample2 = -(myICM.*_gyro_axis)() * TO_RAD;
    delay(30);
    double sample3 = -(myICM.*_gyro_axis)() * TO_RAD;

    double average = (sample1 + sample2 + sample3) / 3;
    _calibrationRotVelZ = average;
    _smoothRotationVelocity = 0;
}

// time until the gyro calibration weighted average is 1/2 of the way to the new value
#define GYRO_CALIBRATE_PERIOD_MS 1000
// the threshold for the gyro to be considered "stationary"
#define CALIBRATE_THRESH_RAD 1000.0 * TO_RAD // 10 degrees/s

#define GYRO_DRIFT_MAX 2 * TO_RAD // degrees/s^2

#define LPF_BETA 1

// time until the accel calibration weighted average is 1/2 of the way to the new value
#define IMU_CALIBRATE_PERIOD_MS 10000
// threshold for the accelerometer to be considered "stationary"
#define CALIBRATE_THRESH_MPSS 1

void IMU::_updateGyro(double deltaTimeMS)
{
    _currRotVelZ = -(myICM.*_gyro_axis)() * TO_RAD;
    _currRotVelZ *= _gyro_scale_factor;
    _currRotVelZ -= _calibrationRotVelZ;

    _smoothRotationVelocity = _smoothRotationVelocity + (LPF_BETA * (_currRotVelZ - _smoothRotationVelocity));

    double avgRotVelZ = (_smoothRotationVelocity + _prevRotVelZ) / 2;
    double gyroNewWeight = deltaTimeMS / GYRO_CALIBRATE_PERIOD_MS;
    float gyroClamp = deltaTimeMS * GYRO_DRIFT_MAX / 1000;

    // if the gyro is stationary, calibrate it
    if (abs(avgRotVelZ) < CALIBRATE_THRESH_RAD)
    {
        float rotationOffset = (avgRotVelZ - _calibrationRotVelZ)*gyroNewWeight;
        rotationOffset = std::max(-gyroClamp, std::min(gyroClamp, rotationOffset));
        _calibrationRotVelZ += rotationOffset;
        //_calibrationRotVelZ = (1 - gyroNewWeight) * _calibrationRotVelZ + gyroNewWeight * clampedRotVelZ;
    }

    // update the rotation
    _rotation += avgRotVelZ * deltaTimeMS / 1000.0;

    // check for inf
    if (abs(_rotation) > 2 * 3.14 * 1000)
    {
        _rotation = 0;
    }

    // update the previous velocity
    _prevRotVelZ = _smoothRotationVelocity;
}

static double magnitude(Point p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}


// time until we reset the velocity to 0 if the accelerometer is stationary
#define RESET_VELOCITY_NO_ACCEL_TIME_MS 50

#define VELOCITY_BLEAD_PERIOD_MS 10000
void IMU::_updateAccelerometer(double deltaTimeMS)
{ 
    // get (unfiltered) accelerometer data and set accel
    _currAcceleration.x = myICM.accX() / (9.81 * 10);
    _currAcceleration.y = myICM.accY() / (9.81 * 10);
    _currAcceleration.z = myICM.accZ() / (9.81 * 10) - 9.81;


    /////////////// ROTATE ACCELERATION BY GYRO //////////////////
    double cosRotation = cos(_rotation);
    double sinRotation = sin(_rotation);
    double rotatedX = _currAcceleration.x * cosRotation - _currAcceleration.y * sinRotation;
    double rotatedY = _currAcceleration.x * sinRotation + _currAcceleration.y * cosRotation;

    _currAcceleration.x = rotatedX;
    _currAcceleration.y = rotatedY;
    //////////////////////////////////////////////////////


    Point avgAccel = (_currAcceleration + _prevAcceleration) / 2;
    double accelNewWeight = deltaTimeMS / IMU_CALIBRATE_PERIOD_MS;

    // if the accelerometer is stationary, calibrate it
    if (magnitude(avgAccel) < CALIBRATE_THRESH_MPSS)
    {
        // move the calibration value towards the new value
        _calibrationAccel = _calibrationAccel * (1 - accelNewWeight) + avgAccel * accelNewWeight;

        // if the accelerometer has been stationary for a while, reset the velocity
        if (millis() - _lastAccelerateTimeMS > RESET_VELOCITY_NO_ACCEL_TIME_MS)
        {
            // if the accelerometer has been stationary for a while, reset the velocity
            _velocity = Point{0, 0, 0};
        }
    }
    else
    {
        // if the accelerometer is not stationary, update the last time we accelerated
        _lastAccelerateTimeMS = millis();

        // update the velocity
        _velocity += (avgAccel - _calibrationAccel) * deltaTimeMS / 1000.0;

        // slowly blead off the velocity to 0 even when we are moving
        double bleadWeight = deltaTimeMS / VELOCITY_BLEAD_PERIOD_MS;
        _velocity = _velocity * (1 - bleadWeight);
    }

    // update the previous acceleration
    _prevAcceleration = _currAcceleration;
}

double currTime = 0;
double prevTime = 0;

/**
 *Updates the time data for the current loop iteration
 */
double getDt()
{
    currTime = micros() / 1000;
    double dt = currTime - prevTime;
    prevTime = currTime;
    return dt;
}

void IMU::Update()
{
    // compute the time since the last update
    double currTimeMS = micros() / 1000;
    double deltaTimeMS = currTimeMS - _prevTimeMS;

    // if the time is too small, don't update
    if (deltaTimeMS < 0.3)
    {
        Serial.println("exiting update not enough time");
        return;
    }

    // get the data from the IMU
    myICM.getAGMT_fast();

    // 1. update the gyro
    _updateGyro(deltaTimeMS);

    uint8_t still_gyros = 0;
    for(int i = 0; i < 4; i++)
    {
        // ignore the local gyro
        if (_placement != i)
        {
            // if we have seen a packet come in from a given gyro at least once every 150ms, and it has reported < 0.1 rads/s for the last 500ms,
            // we can be reasonably confident it thinks the robot is currently still. track how many gyros think the robot is currently still
            const bool isGyroSyncUpdated = (millis() - lastPacketTimestamp[i] < STALE_ANGLE_SYNC_TIMEOUT);
            if (isGyroSyncUpdated == false)
            {
                stoppedMovingTimestamp[i] = 0;
            }

            const bool hasGyroReportedStill = (stoppedMovingTimestamp[i] != 0 && (millis() - stoppedMovingTimestamp[i]) > STILL_THRESHOLD_TIMEOUT);

            if (isGyroSyncUpdated & hasGyroReportedStill)
            {
                still_gyros++;
            }
        }
    }

    if (fabs(_currRotVelZ) < 0.1f)
    {
        if (stoppedMovingTimestamp[_placement] == 0)
        {
            stoppedMovingTimestamp[_placement] = millis();
        }
    }
    else
    {
        stoppedMovingTimestamp[_placement] = 0;
    }

    if (fabs(_currRotVelZ) > 0.15f)
    {
        if (startedMovingTimestamp == 0)
        {
            startedMovingTimestamp = millis();
        }
    }
    else
    {
        startedMovingTimestamp = 0;
    }



    if (still_gyros >= 2)
    {
        // if at least 2 other gyros are confident the robot is still but the onboard gyro thinks robot is moving, mark the local gyro as unhealthy
        if ((startedMovingTimestamp != 0) && (millis() - startedMovingTimestamp > STILL_THRESHOLD_TIMEOUT))
        {
            _imu_healthy = false;
            ForceCalibrate();
        }

        // if we have also been still for the last 500ms, mark the imu as healthy again
        else if ((stoppedMovingTimestamp[_placement] != 0) && (millis() - stoppedMovingTimestamp[_placement] > STILL_THRESHOLD_TIMEOUT))
        {
            _imu_healthy = true;
        }
    }

    // 2. update the accelerometer
    _updateAccelerometer(deltaTimeMS);

    _prevTimeMS = currTimeMS;
}

bool IMU::dataReady()
{
    return myICM.dataReady();
}

/**
 * Gets the acceleration of the robot
 */
Point IMU::getAccel()
{
    return _currAcceleration - _calibrationAccel;
}

/**
 * Gets the velocity of the robot
 */
Point IMU::getVelocity()
{
    return _velocity;
}

/**
 * Gets the rotational velocity of the robot in radians per second
 */
double IMU::getRotationVelocity()
{
    return _currRotVelZ - _calibrationRotVelZ;
}

/**
 * Gets the rotation of the robot in radians
 */
double IMU::getRotation()
{
    return _rotation;
}

void IMU::plotData(double orient, double vel, double accel)
{
    Serial.print("Orientation:");
    Serial.print(orient);
    Serial.print(",");
    Serial.print("Velocity:");
    Serial.print(vel);
    Serial.print(",");
    Serial.print("Acceleration:");
    Serial.println(accel);
}

void IMU::printScaledAGMT()
{
    Serial.print("Scaled. Acc (mg) [ ");
    printFormattedFloat(myICM.accX(), 5, 2);
    Serial.print(", ");
    printFormattedFloat(myICM.accY(), 5, 2);
    Serial.print(", ");
    printFormattedFloat(myICM.accZ(), 5, 2);
    Serial.print(" ], Gyr (DPS) [ ");
    printFormattedFloat(myICM.gyrX(), 5, 2);
    Serial.print(", ");
    printFormattedFloat(myICM.gyrY(), 5, 2);
    Serial.print(", ");
    printFormattedFloat(myICM.gyrZ(), 5, 2);
    Serial.print(" ], Mag (uT) [ ");
    printFormattedFloat(myICM.magX(), 5, 2);
    Serial.print(", ");
    printFormattedFloat(myICM.magY(), 5, 2);
    Serial.print(", ");
    printFormattedFloat(myICM.magZ(), 5, 2);
    Serial.print(" ], Tmp (C) [ ");
    printFormattedFloat(myICM.temp(), 5, 2);
    Serial.print(" ]");
    Serial.println();
}

void IMU::printPaddedInt16b(int16_t val)
{
    if (val > 0)
    {
        Serial.print(" ");
        if (val < 10000)
        {
            Serial.print("0");
        }
        if (val < 1000)
        {
            Serial.print("0");
        }
        if (val < 100)
        {
            Serial.print("0");
        }
        if (val < 10)
        {
            Serial.print("0");
        }
    }
    else
    {
        Serial.print("-");
        if (abs(val) < 10000)
        {
            Serial.print("0");
        }
        if (abs(val) < 1000)
        {
            Serial.print("0");
        }
        if (abs(val) < 100)
        {
            Serial.print("0");
        }
        if (abs(val) < 10)
        {
            Serial.print("0");
        }
    }
    Serial.print(abs(val));
}

void IMU::printRawAGMT(ICM_20948_AGMT_t agmt)
{
    Serial.print("RAW. Acc [ ");
    printPaddedInt16b(agmt.acc.axes.x);
    Serial.print(", ");
    printPaddedInt16b(agmt.acc.axes.y);
    Serial.print(", ");
    printPaddedInt16b(agmt.acc.axes.z);
    Serial.print(" ], Gyr [ ");
    printPaddedInt16b(agmt.gyr.axes.x);
    Serial.print(", ");
    printPaddedInt16b(agmt.gyr.axes.y);
    Serial.print(", ");
    printPaddedInt16b(agmt.gyr.axes.z);
    Serial.print(" ], Mag [ ");
    printPaddedInt16b(agmt.mag.axes.x);
    Serial.print(", ");
    printPaddedInt16b(agmt.mag.axes.y);
    Serial.print(", ");
    printPaddedInt16b(agmt.mag.axes.z);
    Serial.print(" ], Tmp [ ");
    printPaddedInt16b(agmt.tmp.val);
    Serial.print(" ]");
    Serial.println();
}

void IMU::printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
    float aval = abs(val);
    if (val < 0)
    {
        Serial.print("-");
    }
    else
    {
        Serial.print(" ");
    }
    for (uint8_t indi = 0; indi < leading; indi++)
    {
        uint32_t tenpow = 0;
        if (indi < (leading - 1))
        {
            tenpow = 1;
        }
        for (uint8_t c = 0; c < (leading - 1 - indi); c++)
        {
            tenpow *= 10;
        }
        if (aval < tenpow)
        {
            Serial.print("0");
        }
        else
        {
            break;
        }
    }
    if (val < 0)
    {
        Serial.print(-val, decimals);
    }
    else
    {
        Serial.print(val, decimals);
    }
}

void IMU::MergeExternalInput(board_placement placement, float rotation, float velocity)
{
    all_velocities[placement] = velocity;
    lastPacketTimestamp[placement] = millis();

    if (fabsf(velocity) < 0.1f)
    {
        if (stoppedMovingTimestamp[placement] == 0)
        {
            stoppedMovingTimestamp[placement] = millis();
        }
    }
    else
    {
        stoppedMovingTimestamp[placement] = 0;
    }

    if (millis() < BOOT_GYRO_MERGE_MS)
    {
        _rotation = rotation;
    }
    else
    {
        double difference = rotation - _rotation;

        //fix wrap-around issues before merging
        while(difference > PI)
        {
            difference -= 2*PI;
        }
        while(difference < -PI)
        {
            difference += 2*PI;
        }

        _rotation += EXTERNAL_GYRO_MERGE_WEIGHT*difference;
    }
}