#include "RobotStateParser.h"

#include <nlohmann/json.hpp>
#include <string>

UnityRobotState RobotStateParser::parse(const std::string &json_string)
{
    nlohmann::json json = nlohmann::json::parse(json_string);
    Point p = {json["robot_velocity"]["x"], json["robot_velocity"]["y"], json["robot_velocity"]["z"]};
    Point p2 = {json["opponent_position"]["x"], json["opponent_position"]["y"], json["opponent_position"]["z"]};

    double o = json["robot_rotation"];
    double rotational_velocity = json["robot_rotation_velocity"];
    double o2 = json["opponent_rotation"];

    double spinner_1_RPM = json["spinner_1_RPM"];
    double spinner_2_RPM = json["spinner_2_RPM"];
    return {p, o, rotational_velocity, p2, o2, spinner_1_RPM, spinner_2_RPM};
}

std::string RobotStateParser::serialize(UnityDriveCommand message)
{
    nlohmann::json j;
    j["drive_amount"] = message.drive_amount;
    j["turn_amount"] = message.turn_amount;
    j["front_weapon_power"] = message.front_weapon_power;
    j["back_weapon_power"] = message.back_weapon_power;

    return j.dump();
}
