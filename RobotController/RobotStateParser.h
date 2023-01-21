#pragma once

#include <string>
#include <nlohmann/json.hpp>

struct Point
{
    double x;
    double y;
    double z;
};


// should match the C# side
struct RobotState
{
    Point robot_position;
    double robot_orientation;
    Point opponent_position;
    double opponent_orientation;
};

struct RobotControllerMessage
{
    double drive_amount;
    double turn_amount;
    std::vector<Point> point_cloud;
};

class RobotStateParser
{
public:
    static RobotState parse(const std::string &json_string);
    static std::string serialize(RobotControllerMessage message);
};