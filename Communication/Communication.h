#pragma once

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

/**
 * Communication.h
 * 
 * This file contains the structs that are used for communication between the
 * driver station and the robot.
*/

struct Point
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    // ctor
    Point(double x, double y, double z) : x(x), y(y), z(z) {}
    // default ctor
    Point() {}

    // operators
    Point operator+(const Point& other) const
    {
        return Point(x + other.x, y + other.y, z + other.z);
    }

    Point operator-(const Point& other) const
    {
        return Point(x - other.x, y - other.y, z - other.z);
    }

    Point operator*(const double& scalar) const
    {
        return Point(x * scalar, y * scalar, z * scalar);
    }

    Point operator/(const double& scalar) const
    {
        return Point(x / scalar, y / scalar, z / scalar);
    }

    // +=
    Point& operator+=(const Point& other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
};

// driver station -> robot
struct DriveCommand
{
    double movement;
    double turn;
    float frontWeaponPower;
    float backWeaponPower;
    float selfRighterPower;
    bool valid; // should always be true, used to check for blank messages
};

// disable padding
#pragma pack(push, 1)

// robot -> driver station
enum RobotMessageType : char
{
    INVALID = 0,
    IMU_DATA,
    CAN_DATA
};

struct IMUData
{
    float rotation;
    float rotationVelocity;
    float accelX;
    float accelY;
};

#define MOTOR_COUNT 4
enum MotorIndexes
{
    l_drive = 0,
    r_drive,
    f_weapon,
    b_weapon
};


// what to multiply the scaled rpm from the package by to get actual rpm
#define ERPM_FIELD_SCALAR 1000.0
#define ERPM_TO_RPM (1.0 / 6.0)
#define RPM_TO_ERPM 6.0

struct CANData
{
    unsigned char motorCurrent[4];
    unsigned char motorVoltage[4];
    unsigned char motorERPM[4];
    unsigned char escFETTemp[4];
};

// TODO: implement me
struct RadioData
{
    short averageDelayMS;
    short maxDelayMS;
    float movement;
    float turn;
    float frontWeaponPower;
    float backWeaponPower;
};

// TODO: implement ping measurement system

// Union that combines RobotMessage and TelemetryMessage
struct RobotMessage
{
    RobotMessageType type;

    union
    {
        IMUData imuData;
        CANData canData;
    };

    bool valid;
};

#pragma pack(pop)

// indicates the end of a message. All communciation between devices should end with this
const char MESSAGE_END_CHAR = '\n';

const char MESSAGE_START_CHAR = '@';


#endif