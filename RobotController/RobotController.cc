
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
    double angle = std::atan2(y2 - y1, x2 - x1);
    return angle < -M_PI_2 ? angle + M_PI : angle;
}

#define TO_RAD (M_PI / 180.0)

RobotControllerMessage loop(RobotState &state)
{
    std::cout << "loop" << std::endl;
    RobotControllerMessage response{0, 0};

    double angle_delta = angle_between_points(state.robot_position.x, state.robot_position.y,
                                              state.opponent_position.x, state.opponent_position.y);

    // turn towards other robot for now
    response.turn_amount = std::clamp(angle_delta / (30.0 * TO_RAD), -1.0, 1.0);
    return response;
}
