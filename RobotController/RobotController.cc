
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
    double angle_delta = angle_between_points(state.robot_position.x, state.robot_position.z,
                                              state.opponent_position.x, state.opponent_position.z);
    angle_delta += angle_wrap(state.robot_orientation * TO_RAD);
    angle_delta = angle_wrap(angle_delta);
    const double max_power_angle = 30.0 * TO_RAD;
    // turn towards other robot for now
    response.turn_amount = -std::clamp(angle_delta / max_power_angle, -1.0, 1.0);
    return response;
}
