// ServerSocket.cpp
#include "ServerSocket.h"
#include <string>
#include <iostream>

/**
 * Constructor
 * @param port The port to listen on
 */
ServerSocket::ServerSocket(std::string port)
    : port(port), listenSocket(INVALID_SOCKET)
{
    setup_receiving_socket();
}

ServerSocket::~ServerSocket()
{
    closesocket(listenSocket);
    WSACleanup();
}

/**
 * Setup the socket to receive messages
 */
int ServerSocket::setup_receiving_socket()
{
    WSADATA wsaData;
    int iResult;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0)
    {
        std::cerr << "WSAStartup failed with error: " << iResult << std::endl;
        return 1;
    }

    SOCKADDR_IN serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(std::stoi(port));
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);

    // Create a SOCKET for listening
    listenSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (listenSocket == INVALID_SOCKET)
    {
        std::cerr << "socket failed with error: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return 1;
    }

    // Bind the listening socket to the specified port
    if (bind(listenSocket, (SOCKADDR *)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR)
    {
        std::cerr << "bind failed with error: " << WSAGetLastError() << std::endl;
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }

    return 0;
}

/**
 * Receive a message from the socket
 * @return The message received, otherwise an empty string
 */
std::string ServerSocket::receive(int* outError)
{
    last_sender_addr_len = sizeof(last_sender_addr);
    std::string ret = "";

    // Set the socket to non-blocking mode
    u_long iMode = 1;
    ioctlsocket(listenSocket, FIONBIO, &iMode);

    // Read all available messages and only keep the last one
    while (true)
    {
        int numBytesReceived = recvfrom(listenSocket, recvbuf, recvbuflen, 0, (SOCKADDR *)&last_sender_addr, &last_sender_addr_len);

        if (numBytesReceived == SOCKET_ERROR)
        {
            int err = WSAGetLastError();

            if (err == WSAEWOULDBLOCK)
            {
                // No more messages
                break;
            }
            else
            {
                // Error occurred
                std::cerr << "recvfrom on port " << port << " failed with error: " << err << std::endl;
                if (outError != nullptr)
                {
                    *outError = err;
                    return "";
                }

                break;

            }
        }
        else if (numBytesReceived > 0)
        {
           
            ret.assign(recvbuf, numBytesReceived);
            // Update last_sender_addr and last_sender_addr_len
            // This is already handled by recvfrom parameters
        }
    }

    // Set the socket back to blocking mode
    iMode = 0;
    ioctlsocket(listenSocket, FIONBIO, &iMode);

    return ret;
}

/**
 * Replies to the last sender
 *
 * @param data The string to send
 */
void ServerSocket::reply_to_last_sender(std::string data)
{
    if((listenSocket == INVALID_SOCKET) || (last_sender_addr_len == 0) || (data.length() > 65400) ) // leave room for unknown overhead
    {
        return;
    }

    int iSendResult = sendto(listenSocket, data.c_str(), data.length(), 0, (SOCKADDR *)&last_sender_addr, last_sender_addr_len);

    // if (iSendResult == SOCKET_ERROR)
    // {
    //     std::cerr << "sendto failed with error: " << WSAGetLastError() << std::endl;
    // }
}
