#include "RobotStateParser.h"

RobotState RobotStateParser::parse(const std::string &json_string)
{
    nlohmann::json json = nlohmann::json::parse(json_string);
    Point p = {json["robot_position"]["x"], json["robot_position"]["y"]};
    double o = json["robot_orientation"];
    return {p, o};
}