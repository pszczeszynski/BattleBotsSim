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