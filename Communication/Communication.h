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
    bool valid; // should always be true, used to check for blank messages
};

// robot -> driver station
struct RobotMessage
{
    double rotation;
    double rotationVelocity;
    Point accel;
    Point velocity;
};

// indicates the end of a message. All communciation between devices should end with this
const char MESSAGE_END_CHAR = '\n';

const char MESSAGE_START_CHAR = '@';





#endif