#pragma once

#pragma comment(lib, "ws2_32.lib")

#include <winsock2.h>
#include <iostream>
#include <ws2tcpip.h>

#define DEFAULT_BUFLEN 512

class ServerSocket
{
private:
    // The port to listen on
    std::string port;
    // The socket to listen on
    SOCKET listenSocket;
    // Buffer for receiving data
    char recvbuf[DEFAULT_BUFLEN];
    // Number of bytes actually received
    int recvbuflen = DEFAULT_BUFLEN;
    // The address of the last sender
    sockaddr_storage last_sender_addr;
    // The length of the last sender's address
    socklen_t last_sender_addr_len;

    int setup_receiving_socket();
public:
    std::string receive();
    void reply_to_last_sender(std::string data);
    ServerSocket(std::string port);
};