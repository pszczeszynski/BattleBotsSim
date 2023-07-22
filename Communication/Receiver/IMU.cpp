#include "IMU.h"

#include <Arduino.h>

IMU::IMU()
{
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    bool initialized = false;
    while (!initialized)
    {
        myICM.begin(WIRE_PORT, AD0_VAL);

        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok)
        {
            Serial.println("Trying again...");
            delay(500);
        }
        else
        {
            initialized = true;
        }
    }

    velocity = {0, 0, 0};
}

void IMU::Update()
{
    myICM.getAGMT();
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
    Point accel = {0, 0, 0};

    // get (unfiltered) accelerometer data and set accel
    
    accel.x = myICM.accX() / (9.8 * 1000) /* m/s^2 */;
    accel.y = myICM.accY() / (9.8 * 1000) /* m/s^2 */;
    accel.z = myICM.accZ() / (9.8 * 1000) /* m/s^2 */;

    return accel;
}

/**
 * Gets the velocity of the robot
 */
Point IMU::getVelocity(Point accel, int dt)
{
    velocity.x += accel.x /* m/s^2 */ * dt / 1000.0 /* s */;
    velocity.y += accel.y /* m/s^2 */ * dt / 1000.0 /* s */;
    velocity.z += accel.z /* m/s^2 */ * dt / 1000.0 /* s */;

    return velocity;
}

/**
 * Gets the rotation of the robot
 */
double IMU::getRotation(int dt)
{
    double rotation = 0;

    //This won't work
    rotation = myICM.gyrZ() * dt / 1000.0;

    return rotation;
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