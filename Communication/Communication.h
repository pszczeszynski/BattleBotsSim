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
    double x;
    double y;
    double z;
};

// driver station -> robot
struct DriveCommand
{
    double movement;
    double turn;
};

// robot -> driver station
struct RobotMessage
{
    Point accel;
    Point velocity;
    double rotation;
};
#endif