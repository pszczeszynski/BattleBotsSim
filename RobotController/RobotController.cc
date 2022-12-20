
#include "ServerSocket.h"
#include "RobotStateParser.h"

int main()
{
    ServerSocket socket{"11115"};

    // receive until the peer closes the connection
    while (true)
    {
        std::string received = socket.receive();
        if (received == "") continue;

        RobotState state = RobotStateParser::parse(received);
        std::cout << "robot_position: " << state.robot_position.x << ", " << state.robot_position.y << std::endl;
        std::cout << "robot_orientation: " << state.robot_orientation << std::endl;

        socket.reply_to_last_sender("We're working on a response, not done yet");
    }
}