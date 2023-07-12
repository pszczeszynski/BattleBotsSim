#include "RobotStateParser.h"

#include <nlohmann/json.hpp>
#include <string>

RobotState RobotStateParser::parse(const std::string &json_string)
{
    nlohmann::json json = nlohmann::json::parse(json_string);
    Point p = {json["robot_velocity"]["x"], json["robot_velocity"]["y"], json["robot_velocity"]["z"]};
    Point p2 = {json["opponent_position"]["x"], json["opponent_position"]["y"], json["opponent_position"]["z"]};

    double o = json["robot_orientation"];
    double o2 = json["opponent_orientation"];
    return {p, o, p2, o2};
}

std::string RobotStateParser::serialize(RobotControllerMessage message)
{
    nlohmann::json j;
    j["drive_amount"] = message.drive_amount;
    j["turn_amount"] = message.turn_amount;

    return j.dump();
}
