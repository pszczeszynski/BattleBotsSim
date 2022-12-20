#pragma once

#pragma comment(lib, "ws2_32.lib")

#include <winsock2.h>
#include <iostream>
#include <ws2tcpip.h>

#define DEFAULT_BUFLEN 512

class ServerSocket
{
private:
    std::string port;
    SOCKET listenSocket;
    int setup_receiving_socket();

    char recvbuf[DEFAULT_BUFLEN];
    int recvbuflen = DEFAULT_BUFLEN;

    sockaddr_storage last_sender_addr;
    socklen_t last_sender_addr_len;

public:
    std::string receive();
    void reply_to_last_sender(std::string);
    ServerSocket(std::string port);
};