#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include "../Common/Communication.h"

/**
 * The state of the robot, should match the C# side.
 * NOTE: this shouldn't be used for navigating the robot.
 * All of the information it needs should be obtained from
 * the vision system.
*/
struct UnityRobotState
{
    // velocity of our robot
    Point robot_position;
    // orientation of our robot
    double robot_rotation;
    // the rotation velocity of our robot in radians per second
    double robot_rotation_velocity;

    // position of the opponent's robot
    Point opponent_position;
    // orientation of the opponent's robot
    double opponent_rotation;
    double opponent_rotation_velocity;

    // spinner rpms
    double spinner_1_RPM;
    double spinner_2_RPM;
};

/**
 * Drives the unity robot
*/
struct UnityDriveCommand
{
    double drive_amount;
    double turn_amount;
    double front_weapon_power;
    double back_weapon_power;
    bool reset;
    double opponent_drive_amount;
    double opponent_turn_amount;
};

class RobotStateParser
{
public:
    static UnityRobotState parse(const std::string &json_string);
    static std::string serialize(UnityDriveCommand message);
};