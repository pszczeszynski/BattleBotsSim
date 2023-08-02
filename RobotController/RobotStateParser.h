#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include "../Communication/Communication.h"

/**
 * The state of the robot, should match the C# side.
 * NOTE: this shouldn't be used for navigating the robot.
 * All of the information it needs should be obtained from
 * the vision system.
*/
struct UnityRobotState
{
    // velocity of our robot
    Point robot_velocity;
    // orientation of our robot
    double robot_orientation;
    // the rotation velocity of our robot in radians per second
    double robot_rotation_velocity;

    // position of the opponent's robot
    Point opponent_position;
    // orientation of the opponent's robot
    double opponent_orientation;
};

/**
 * Drives the unity robot
*/
struct UnityDriveCommand
{
    double drive_amount;
    double turn_amount;
};

class RobotStateParser
{
public:
    static UnityRobotState parse(const std::string &json_string);
    static std::string serialize(UnityDriveCommand message);
};