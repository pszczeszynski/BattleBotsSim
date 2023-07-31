#include "IMU.h"

#include <Arduino.h>

IMU::IMU()
{
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    // attempt to initialize the imu in a loop until it works
    bool initialized = false;
    while (!initialized)
    {
        myICM.begin(WIRE_PORT, AD0_VAL);

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

    // take an initial reading for the z velocity calibration
    zVelocityCalibration = myICM.gyrZ();

    // initialize the velocity and rotation
    velocity = {0, 0, 0};
    rotation = 0.0;
}

// time until the gyro calibration weighted average is 1/2 of the way to the new value
#define GYRO_CALIBRATE_PERIOD_MS 10000
// the threshold for the gyro to be considered "stationary"
#define CALIBRATE_THRESH_RAD 4 * TO_RAD

// time until the accel calibration weighted average is 1/2 of the way to the new value
#define IMU_CALIBRATE_PERIOD_MS 10000
// threshold for the accelerometer to be considered "stationary"
#define CALIBRATE_THRESH_MPSS 0.1

void IMU::Update(double deltaTimeMS)
{
    // get the data from the IMU
    myICM.getAGMT();

    /////////////////// GYRO LOGIC ///////////////////
    currRotVelZ = -myICM.gyrZ() * TO_RAD;
    avgRotVelZ = (currRotVelZ + prevRotVelZ) / 2;
    double gyroNewWeight = deltaTimeMS / GYRO_CALIBRATE_PERIOD_MS;

    // if the gyro is stationary, calibrate it
    if (abs(avgRotVelZ) < CALIBRATE_THRESH_RAD)
    {
        zVelocityCalibration = (1 - gyroNewWeight) * zVelocityCalibration + gyroNewWeight * avgRotVelZ;
    }

    // update the rotation
    rotation += (avgRotVelZ - zVelocityCalibration) * deltaTimeMS / 1000.0;
    // update the previous velocity
    prevRotVelZ = currRotVelZ;
    ///////////////////////////////////////////////////

    /////////////////// ACCEL LOGIC ///////////////////
    // get (unfiltered) accelerometer data and set accel
    acceleration.x = myICM.accX() / (9.81 * 1000);
    acceleration.y = myICM.accY() / (9.81 * 1000);
    acceleration.z = myICM.accZ() / (9.81 * 1000);
    ///////////////////////////////////////////////////


    ////////////////// VEL LOGIC //////////////////////
    velocity.x += acceleration.x /* m/s^2 */ * deltaTimeMS / 1000.0 /* s */;
    velocity.y += acceleration.y /* m/s^2 */ * deltaTimeMS / 1000.0 /* s */;
    velocity.z += acceleration.z /* m/s^2 */ * deltaTimeMS / 1000.0 /* s */;
    ///////////////////////////////////////////////////
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
    return acceleration;
}

/**
 * Gets the velocity of the robot
 */
Point IMU::getVelocity()
{
    return velocity;
}

/**
 * Gets the rotational velocity of the robot in radians per second
 */
double IMU::getRotationVelocity()
{
    return currRotVelZ - zVelocityCalibration;
}

/**
 * Gets the rotation of the robot in radians
 */
double IMU::getRotation()
{
    return rotation;
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