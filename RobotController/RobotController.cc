
#include "ServerSocket.h"

int main()
{
    ServerSocket socket{"11115"};
    // receive until the peer closes the connection
    while (true)
    {
        std::string received = socket.receive();
        std::cout << "received: " << received << std::endl;
        if (received == "") continue;
        socket.reply_to_last_sender(received);
    }
}