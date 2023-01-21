#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

class ServerSocket
{
private:
    int socket_fd;
    struct sockaddr_in server_address;
    struct sockaddr_in last_sender_address;
    socklen_t sender_address_len;

public:
    std::string receive();
    void reply_to_last_sender(std::string data);
    ServerSocket(std::string port);
}