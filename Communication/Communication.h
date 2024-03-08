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

// defines the radio channel indexes for each teensy
#define TEENSY_RADIO_1 52
#define TEENSY_RADIO_2 47
#define TEENSY_RADIO_3 57

// disable padding
#pragma pack(push, 1)

// robot -> driver station
enum RobotMessageType : char
{
    INVALID = 0,
    IMU_DATA,
    CAN_DATA,
    RADIO_DATA,
    CHANNEL_SWITCH
};

struct IMUData
{
    double rotation;
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
#define ERPM_TO_RPM (1.0 / 8.0)
#define RPM_TO_ERPM 8.0

struct CANData
{
    unsigned char motorCurrent[4];
    unsigned char motorVoltage[4];
    unsigned char motorERPM[4];
    unsigned char escFETTemp[4];
    unsigned char motorTemp[4];
};

struct RadioData
{
    short averageDelayMS;
    short maxDelayMS;
    float movement;
    float turn;
    float frontWeaponPower;
    float backWeaponPower;
    short invalidPackets;
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
        RadioData radioData;
    };

    bool valid;
};

// specifies coefficients for the autonomous go to angle
struct AutoDrive
{
    float movement;
    float targetAngle; // SHOULD BE ADJUSTED FOR THIS TEENSY (is not a global angle)
    short ANGLE_EXTRAPOLATE_MS;
    short TURN_THRESH_1_DEG;
    short TURN_THRESH_2_DEG;
    short MAX_TURN_POWER_PERCENT;
    short MIN_TURN_POWER_PERCENT;
    short SCALE_DOWN_MOVEMENT_PERCENT;
    bool invertTurn;
    short frontWeaponPower;
    short backWeaponPower;
};

// driver station -> robot
struct DriveCommand
{
    float movement;
    float turn;
    float frontWeaponPower;
    float backWeaponPower;
    float selfRighterPower;
};

enum DriverStationMessageType : char
{
    INVALID_DS = 0,
    DRIVE_COMMAND,
    AUTO_DRIVE
};

struct DriverStationMessage
{
    DriverStationMessageType type;
    unsigned char radioChannel;

    union
    {
        DriveCommand driveCommand;
        AutoDrive autoDrive;
    };

    bool valid; // should always be true, used to check for blank messages
};

#pragma pack(pop)

// indicates the end of a message. All communciation between devices should end with this
const char MESSAGE_END_CHAR = '\n';

const char MESSAGE_START_CHAR = '@';


#endif