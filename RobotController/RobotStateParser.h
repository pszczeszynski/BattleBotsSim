#pragma once

#include <string>
#include <nlohmann/json.hpp>

struct Point
{
    double x;
    double y;
};

struct RobotState
{
    Point robot_position;
    double robot_orientation;
};

class RobotStateParser
{
public:
    static RobotState parse(const std::string &json_string);
};