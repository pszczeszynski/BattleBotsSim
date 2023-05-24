#include "ServerSocket.h"
#include <fcntl.h>

/**
 * Constructor
 * @param port The port to listen on
 * @return A ServerSocket object
*/
ServerSocket::ServerSocket(std::string port)
    : port(port), listenSocket(INVALID_SOCKET)
{
    setup_receiving_socket();
}

/**
 * Setup the socket to receive messages
 * This is only internal and is called by the constructor.
 * @return 0 on success, otherwise 1
*/
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

/**
 * Receive a message from the socket
 * @return The message received, otherwise an empty string
*/
std::string ServerSocket::receive()
{
    last_sender_addr_len = sizeof(last_sender_addr);
    std::string ret = "";

    // Set the socket to non-blocking mode
    u_long iMode = 1;
    ioctlsocket(listenSocket, FIONBIO, &iMode);

    // Read all available messages and only keep the last one
    while (true)
    {
        int numBytesReceived = recvfrom(listenSocket, recvbuf, recvbuflen, 0,
                                        (sockaddr *)&last_sender_addr, &last_sender_addr_len);

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
                return "";
            }
        }

        ret.assign(recvbuf, numBytesReceived);
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
 * @return void
*/
void ServerSocket::reply_to_last_sender(std::string data)
{
    int iSendResult = sendto(listenSocket, data.c_str(), data.length(), 0, (sockaddr *)&last_sender_addr, last_sender_addr_len);

    if (iSendResult == SOCKET_ERROR)
    {
        std::cerr << "sendto failed with error: " << WSAGetLastError() << std::endl;
    }
}
