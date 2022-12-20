#include <winsock2.h>
#include <iostream>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

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

    ServerSocket(std::string port) : port(port), listenSocket(INVALID_SOCKET)
    {
        setup_receiving_socket();
    }
};

int ServerSocket::setup_receiving_socket()
{
    WSADATA wsaData;
    int iResult;

    addrinfo *result = NULL;
    addrinfo hints;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0)
    {
        std::cerr << "WSAStartup failed with error: " << iResult << std::endl;
        return 1;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port. Returns 0 on success or an error
    iResult = getaddrinfo(NULL, port.c_str(), &hints, &result);
    if (iResult != 0)
    {
        std::cerr << "getaddrinfo failed with error: " << iResult << std::endl;
        WSACleanup();
        return 1;
    }

    // Create a SOCKET for connecting to server
    listenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (listenSocket == INVALID_SOCKET)
    {
        std::cerr << "socket failed witherror: " << WSAGetLastError() << std::endl;
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }

    // Setup the UDP listening socket
    iResult = bind(listenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR)
    {
        std::cerr << "bind failed with error: " << WSAGetLastError() << std::endl;
        freeaddrinfo(result);
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }

    freeaddrinfo(result);

    return 0;
}

std::string ServerSocket::receive()
{
    last_sender_addr_len = sizeof(last_sender_addr);
    int numBytesReceived = recvfrom(listenSocket, recvbuf, recvbuflen, 0,
                                    (sockaddr *)&last_sender_addr, &last_sender_addr_len);

    std::string ret = "";
    if (numBytesReceived > 0)
    {
        ret.assign(recvbuf, numBytesReceived);
    }

    return ret;
}

void ServerSocket::reply_to_last_sender(std::string data) {

    int iSendResult = sendto(listenSocket, data.c_str(), data.length(), 0, (sockaddr *)&last_sender_addr, last_sender_addr_len);
    if (iSendResult == SOCKET_ERROR)
    {
        std::cerr << "sendto failed with error: " << WSAGetLastError() << std::endl;
    }
}

int main()
{
    ServerSocket socket{"11115"};
    // receive until the peer closes the connection
    while (true)
    {
        std::string received = socket.receive();
        if (received == "") continue;
        socket.reply_to_last_sender(received);
    }
}