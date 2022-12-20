#include "RobotStateParser.h"

#include <nlohmann/json.hpp>
#include <string>

RobotState RobotStateParser::parse(const std::string &json_string)
{
    nlohmann::json json = nlohmann::json::parse(json_string);
    Point p = {json["position"]["x"], json["position"]["y"]};
    double o = json["orientation"];
    return {p, o};
}

std::string RobotStateParser::serialize(RobotControllerMessage message)
{
    nlohmann::json j;
    j["drive_amount"] = message.drive_amount;
    j["turn_amount"] = message.turn_amount;

    return j.dump();
}
