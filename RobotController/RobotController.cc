
#include "ServerSocket.h"
#include "RobotStateParser.h"
#include <cmath>

RobotControllerMessage loop(RobotState &);

int main()
{
    ServerSocket socket{"11115"};

    // receive until the peer closes the connection
    while (true)
    {
        std::string received = socket.receive();
        if (received == "")
        {
            continue;
        }
        RobotState state = RobotStateParser::parse(received);
        RobotControllerMessage response = loop(state);
        socket.reply_to_last_sender(RobotStateParser::serialize(response));
    }
}

// TODO: move this to a better place
double angle_between_points(double x1, double y1, double x2, double y2)
{
    return std::atan2(y2 - y1, x2 - x1);
}

double angle_wrap(double angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle <= -M_PI)
    {
        angle += 2 * M_PI;
    }
    else if (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    return angle;
}


#define TO_RAD (M_PI / 180.0)
#define TO_DEG (180.0 / M_PI)
RobotControllerMessage loop(RobotState &state)
{
    RobotControllerMessage response{0, 0};

    // angle from opponent to us
    double angle_opponent_to_us = angle_between_points(state.opponent_position.x, state.opponent_position.z,
                                              state.robot_position.x, state.robot_position.z);

    // std::cout << "angle_opponent_to_us: " << angle_opponent_to_us * TO_DEG << std::endl;
    const double FOLLOW_DIST_RAD = 30 * TO_RAD;
    const double ORBIT_RADIUS_M = 2;
    double angle_to_target_from_robot = angle_wrap(angle_opponent_to_us + FOLLOW_DIST_RAD);
    
    double follow_point_x = state.opponent_position.x + cos(angle_to_target_from_robot) * ORBIT_RADIUS_M;
    double follow_point_z = state.opponent_position.z + sin(angle_to_target_from_robot) * ORBIT_RADIUS_M;

    // std::cout << "opponent_position.x: " << state.opponent_position.x * TO_DEG << std::endl;
    // std::cout << "opponent_position.z: " << state.opponent_position.z * TO_DEG << std::endl;
    // std::cout << "robot_position.x: " << state.robot_position.x * TO_DEG << std::endl;
    // std::cout << "robot_position.z: " << state.robot_position.z * TO_DEG << std::endl;

    // std::cout << "angle_to_target_from_robot: " << angle_to_target_from_robot * TO_DEG << std::endl;

    // std::cout << "follow_point_x: " << follow_point_x << std::endl;
    // std::cout << "follow_point_z: " << follow_point_z << std::endl;

    // angle from us to the follow point
    double angle_us_to_follow_point = angle_between_points(state.robot_position.x, state.robot_position.z,
                                       follow_point_x, follow_point_z);
    // std::cout << "angle_us_to_follow_point: " << angle_us_to_follow_point * TO_DEG << std::endl;


    angle_us_to_follow_point += angle_wrap(state.robot_orientation * TO_RAD);
    angle_us_to_follow_point = angle_wrap(angle_us_to_follow_point);
    const double max_power_angle = 30.0 * TO_RAD;
    // turn towards other robot for now
    response.turn_amount = -std::clamp(angle_us_to_follow_point / max_power_angle, -1.0, 1.0);
    response.drive_amount = -1.0;
    return response;
}
