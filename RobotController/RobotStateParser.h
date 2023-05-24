#pragma once

#include <string>
#include <nlohmann/json.hpp>

struct Point
{
    double x;
    double y;
    double z;
};

/**
 * The state of the robot, should match the C# side.
 * NOTE: this shouldn't be used for navigating the robot.
 * All of the information it needs should be obtained from
 * the vision system.
*/
struct RobotState
{
    // position of our robot
    Point robot_position;
    // orientation of our robot
    double robot_orientation;
    // position of the opponent's robot
    Point opponent_position;
    // orientation of the opponent's robot
    double opponent_orientation;
};

struct RobotControllerMessage
{
    double drive_amount;
    double turn_amount;
};

class RobotStateParser
{
public:
    static RobotState parse(const std::string &json_string);
    static std::string serialize(RobotControllerMessage message);
};