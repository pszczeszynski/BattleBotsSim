
#include "ServerSocket.h"
#include "RobotStateParser.h"

int main()
{
    ServerSocket socket{"11115"};

    
    // receive until the peer closes the connection
    while (true)
    {
        std::string received = socket.receive();
        if (received == "") { continue; }
        std::cout << "received: " << received << std::endl;
        RobotState state = RobotStateParser::parse(received);
        std::cout << "robot_position: " << state.robot_position.x << ", " << state.robot_position.y << std::endl;
        std::cout << "robot_orientation: " << state.robot_orientation << std::endl;
        
        RobotControllerMessage response{0.0, 0.0};

        if (state.robot_orientation > 90) {
            response.turn_amount = 1.0;
        } else {
            response.turn_amount = 0.0;
        }


        socket.reply_to_last_sender(RobotStateParser::serialize(response));
    }
}