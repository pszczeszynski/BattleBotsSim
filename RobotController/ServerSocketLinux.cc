#include "ServerSocketLinux.h"

ServerSocket::ServerSocket(std::string port)
{
    // Create the socket file descriptor
    if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        std::cerr << "Failed to create socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Clear and set the server address data
    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = INADDR_ANY;
    server_address.sin_port = htons(std::stoi(port));

    // Bind the socket to the server address
    if (bind(socket_fd, (const struct sockaddr *)&server_address, sizeof(server_address)) < 0)
    {
        std::cerr << "Failed to bind socket" << std::endl;
        exit(EXIT_FAILURE);
    }
}

std::string ServerSocket::receive()
{
    // Clear the buffer and set the sender address length
    char buffer[1024] = {0};
    sender_address_len = sizeof(last_sender_address);

    // Receive data from the socket
    int n = recvfrom(socket_fd, (char *)buffer, sizeof(buffer), 0, (struct sockaddr *)&last_sender_address, &sender_address_len);

    // Return the received data as a string
    return std::string(buffer, n);
}

void ServerSocket::reply_to_last_sender(std::string data)
{
    // send the data to the last sender
    sendto(socket_fd, data.c_str(), data.length(), 0, (struct sockaddr *)&last_sender_address, sender_address_len);
}
