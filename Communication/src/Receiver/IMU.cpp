#include "Receiver/IMU.h"
#include "Hardware.h"

#include <Arduino.h>

#include <cmath>

#define CENTER_GYRO_SCALE_FACTOR -1.0
#define CENTER_GYRO_AXIS gyrX

#define SIDE_GYRO_SCALE_FACTOR -1.0
#define SIDE_GYRO_AXIS gyrZ


#define PI 3.14159265358979323846



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
    _placement = placement;
    
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


void IMU::_updateGyro(double deltaTimeMS)
{
    // the threshold for the gyro to be considered "stationary"
    constexpr double kCalibrateThreshRadsPerSec = 10.0 * TO_RAD; // 10 degrees/s
    // lower betas mean more velocity smoothing. Lower betas are faster
    constexpr double kLPFRotVelBeta = 1;
    // time until the gyro calibration weighted average is 1/2 of the way to the new value
    constexpr double kGyroCalibratePeriodMs = 1000;

    _currRotVelZ = -(myICM.*_gyro_axis)() * TO_RAD;
    _currRotVelZ *= _gyro_scale_factor;
    _currRotVelZ -= _calibrationRotVelZ;
    _smoothRotationVelocity = _smoothRotationVelocity + (kLPFRotVelBeta * (_currRotVelZ - _smoothRotationVelocity));
    // assume this update's velocity was constant at the average of the previous and current velocity
    double avgRotVelZ = (_smoothRotationVelocity + _prevRotVelZ) / 2;

    // if the gyro is stationary, calibrate it
    if (abs(avgRotVelZ) < kCalibrateThreshRadsPerSec)
    {
        double gyroNewWeight = deltaTimeMS / kGyroCalibratePeriodMs;
        _calibrationRotVelZ = (1 - gyroNewWeight) * _calibrationRotVelZ + gyroNewWeight * avgRotVelZ;
    }

    // update the rotation
    _rotation += avgRotVelZ * deltaTimeMS / 1000.0;

    // check for inf
    constexpr double kMaxRotationRad = 2 * PI * 1000;
    if (abs(_rotation) > kMaxRotationRad)
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

void IMU::_updateAccelerometer(double deltaTimeMS)
{ 
    // time until the accel calibration weighted average is 1/2 of the way to the new value
    constexpr double kAccelCalibratePeriodMs = 10000;
    // threshold for the accelerometer to be considered "stationary"
    constexpr double kCalibrateThreshMpss = 1;
    // time until we reset the velocity to 0 if the accelerometer is stationary
    constexpr double kResetVelocityNoAccelTimeMs = 50;  
    // time until we slowly blead off the velocity to 0 even when we are moving
    constexpr double kVelocityBleadPeriodMs = 10000;
    
    // get (unfiltered) accelerometer data and set accel
    constexpr double kGravityMpss = 9.81;
    constexpr double kAccelScaleFactor = kGravityMpss * 10;
    
    _currAcceleration.x = myICM.accX() / kAccelScaleFactor;
    _currAcceleration.y = myICM.accY() / kAccelScaleFactor;
    _currAcceleration.z = myICM.accZ() / kAccelScaleFactor - kGravityMpss;


    /////////////// ROTATE ACCELERATION BY GYRO //////////////////
    double cosRotation = cos(_rotation);
    double sinRotation = sin(_rotation);
    double rotatedX = _currAcceleration.x * cosRotation - _currAcceleration.y * sinRotation;
    double rotatedY = _currAcceleration.x * sinRotation + _currAcceleration.y * cosRotation;

    _currAcceleration.x = rotatedX;
    _currAcceleration.y = rotatedY;
    //////////////////////////////////////////////////////


    Point avgAccel = (_currAcceleration + _prevAcceleration) / 2;
    double accelNewWeight = deltaTimeMS / kAccelCalibratePeriodMs;

    // if the accelerometer is stationary, calibrate it
    if (magnitude(avgAccel) < kCalibrateThreshMpss)
    {
        // move the calibration value towards the new value
        _calibrationAccel = _calibrationAccel * (1 - accelNewWeight) + avgAccel * accelNewWeight;

        // if the accelerometer has been stationary for a while, reset the velocity
        if (millis() - _lastAccelerateTimeMS > kResetVelocityNoAccelTimeMs)
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
        double bleadWeight = deltaTimeMS / kVelocityBleadPeriodMs;
        _velocity = _velocity * (1 - bleadWeight);
    }

    // update the previous acceleration
    _prevAcceleration = _currAcceleration;
}

void IMU::Update()
{
    constexpr double kStaleAngleSyncTimeoutMs = 150;
    constexpr double kStillThresholdTimeoutMs = 500;

    // compute the time since the last update
    double currTimeMS = micros() / 1000;
    double deltaTimeMS = currTimeMS - _prevTimeMS;
    _prevTimeMS = currTimeMS;

    // if the time is too small, don't update
    constexpr double kMinUpdateTimeMs = 0.3;
    if (deltaTimeMS < kMinUpdateTimeMs)
    {
        Serial.println("exiting update not enough time");
        return;
    }

    // get the data from the IMU
    myICM.getAGMT_fast();

    // 1. update the gyro
    _updateGyro(deltaTimeMS);

    // 2. update health detection
    _updateHealthDetection();

    // 2. update the accelerometer
    _updateAccelerometer(deltaTimeMS);
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

void IMU::MergeExternalInput(board_placement placement, float rotation,
                             float rotation_velocity) {                           
  // only merge external inputs from the drive and weapon boards
  if (placement != rxDriveLeft && placement != rxDriveRight &&
      placement != rxWepFront && placement != rxWepRear) {
    return;
  }

  constexpr double kExternalGyroMergeWeight = 0.005;

  _all_velocities[placement] = rotation_velocity;
  _lastPacketTimestamps[placement] = millis();

  if (millis() < BOOT_GYRO_MERGE_MS) {
    _rotation = rotation;
  } else {
    double difference = rotation - _rotation;

    // Angle wrap-around
    difference = fmod(difference + PI, 2 * PI) - PI;

    _rotation += kExternalGyroMergeWeight * difference;
  }
}

void IMU::_updateHealthDetection()
{
    constexpr double kVelocityDifferenceThreshold = 0.3; // rad/s average difference threshold
    constexpr double kMaxPercentageError = 20.0; // ±20% error threshold
    constexpr double kVelocityDiffTimeConstantMs = 1000; // 1 second time constant for velocity difference moving average

    const uint32_t current_time = millis();
    
    uint8_t active_external_gyros = _countActiveExternalGyros(current_time);
    
    // Calculate average velocity difference with external gyros
    double current_avg_velocity_diff = _calculateAverageVelocityDifference(current_time, active_external_gyros);
    
    // Update moving average of velocity differences
    if (_last_velocity_diff_update_time != 0)
    {
        double delta_time_ms = current_time - _last_velocity_diff_update_time;
        double alpha = delta_time_ms / kVelocityDiffTimeConstantMs;
        alpha = min(alpha, 1.0); // Clamp to prevent overshooting
        
        _avg_velocity_difference = (1.0 - alpha) * _avg_velocity_difference + alpha * current_avg_velocity_diff;
    }
    else
    {
        _avg_velocity_difference = current_avg_velocity_diff;
    }
    _last_velocity_diff_update_time = current_time;
    
    // Use the smoothed average velocity difference directly as health indicator
    
    // Calculate external mean velocity and percentage error for debug data
    double external_mean_velocity = _calculateExternalMeanVelocity(current_time, active_external_gyros);
    double percentage_error = 0.0;
    
    if (fabs(external_mean_velocity) > 0.01) // Avoid division by very small numbers
    {
        percentage_error = (_avg_velocity_difference / external_mean_velocity) * 100.0;
    }
    
    // Populate debug data
    _imuDebugData.myCurrRotationRad = _rotation;
    _imuDebugData.avgVelocityDifference = _avg_velocity_difference;
    _imuDebugData.externalMeanVelocity = external_mean_velocity;
    _imuDebugData.percentageError = percentage_error;
    
    if (active_external_gyros >= 2)
    {
        // Check both absolute difference and percentage error thresholds
        bool absolute_threshold_ok = (abs(_avg_velocity_difference) < kVelocityDifferenceThreshold);
        bool percentage_threshold_ok = (abs(percentage_error) < kMaxPercentageError);
        
        _imu_healthy = absolute_threshold_ok || percentage_threshold_ok;
    }
    else
    {
        // No external gyros to compare against, assume healthy
        _imu_healthy = true;
    }
    
    _imuDebugData.isHealthy = _imu_healthy;
}

uint8_t IMU::_countActiveExternalGyros(uint32_t current_time)
{
    constexpr uint32_t kStaleAngleSyncTimeoutMs = 150; // 150ms timeout for stale data
    
    uint8_t count = 0;
    
    for (int i = 0; i < 4; i++)
    {
        // Skip local gyro
        if (_placement == i) continue;
        
        // Check if this gyro's data is recent
        if (current_time - _lastPacketTimestamps[i] < kStaleAngleSyncTimeoutMs)
        {
            count++;
        }
    }
    
    return count;
}

double IMU::_calculateAverageVelocityDifference(uint32_t current_time, uint8_t active_external_gyros)
{
    constexpr uint32_t kStaleAngleSyncTimeoutMs = 150; // 150ms timeout for stale data
    
    if (active_external_gyros == 0)
    {
        return 0.0; // No external gyros to compare against
    }
    
    double total_difference = 0.0;
    uint8_t valid_comparisons = 0;
    
    for (int i = 0; i < 4; i++)
    {
        // Skip local gyro
        if (_placement == i) continue;
        
        // Check if this gyro's data is recent
        if (current_time - _lastPacketTimestamps[i] < kStaleAngleSyncTimeoutMs)
        {
            // Calculate velocity difference (signed, not absolute)
            double velocity_difference = _currRotVelZ - _all_velocities[i];
            total_difference += velocity_difference;
            valid_comparisons++;
        }
    }
    
    // Return average difference (signed, so we can detect systematic bias)
    return valid_comparisons > 0 ? total_difference / valid_comparisons : 0.0;
}

double IMU::_calculateExternalMeanVelocity(uint32_t current_time, uint8_t active_external_gyros)
{
    constexpr uint32_t kStaleAngleSyncTimeoutMs = 150; // 150ms timeout for stale data
    
    if (active_external_gyros == 0)
    {
        return 0.0; // No external gyros to calculate mean from
    }
    
    double total_velocity = 0.0;
    uint8_t valid_velocities = 0;
    
    for (int i = 0; i < 4; i++)
    {
        // Skip local gyro
        if (_placement == i) continue;
        
        // Check if this gyro's data is recent
        if (current_time - _lastPacketTimestamps[i] < kStaleAngleSyncTimeoutMs)
        {
            total_velocity += _all_velocities[i];
            valid_velocities++;
        }
    }

    // Return mean velocity of external gyros
    return valid_velocities > 0 ? total_velocity / valid_velocities : 0.0;
}

IMUDebugData IMU::getDebugData()
{
    return _imuDebugData;
}
