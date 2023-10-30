#include "ServerSocket.h"
#include <string>

#ifdef WINDOWS
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
    listenSocket = socket(AF_INET, SOCK_DGRAM, 0);
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
        int numBytesReceived = recvfrom(listenSocket, recvbuf, recvbuflen, 0, &last_sender_addr, &last_sender_addr_len);

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
    int iSendResult = sendto(listenSocket, data.c_str(), data.length(), 0, (SOCKADDR *)&last_sender_addr, last_sender_addr_len);

    if (iSendResult == SOCKET_ERROR)
    {
        std::cerr << "sendto failed with error: " << WSAGetLastError() << std::endl;
    }
}

#else

#include <unistd.h>
#include <arpa/inet.h>

ServerSocket::ServerSocket(std::string port) : port(port), listenSocket(-1)
{
    setup_receiving_socket();
}

int ServerSocket::setup_receiving_socket()
{
    // Initialize the socket address structure
    sockaddr_in serverAddress;
    //std::memset(&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(std::stoi(port));
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);

    // Create a UDP socket for listening
    listenSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (listenSocket == -1)
    {
        std::cerr << "socket creation failed" << std::endl;
        return 1;
    }

    // Bind the socket to the specified port
    if (bind(listenSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1)
    {
        std::cerr << "bind failed" << std::endl;
        close(listenSocket);
        return 1;
    }

    return 0;
}

std::string ServerSocket::receive()
{
    last_sender_addr_len = sizeof(last_sender_addr);
    std::string ret = "";

    fcntl(listenSocket, F_SETFL, O_NONBLOCK);

    // Read all available messages and only keep the last one

    while (true)
    {
        int numBytesReceived = recvfrom(listenSocket, recvbuf, recvbuflen, 0, (struct sockaddr *)&last_sender_addr, &last_sender_addr_len);

        if (numBytesReceived == -1)
        {
            if (errno == EWOULDBLOCK || errno == EAGAIN)
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

    return ret;
}

void ServerSocket::reply_to_last_sender(std::string data)
{
    int iSendResult = sendto(listenSocket, data.c_str(), data.length(), 0, (struct sockaddr *)&last_sender_addr, last_sender_addr_len);

    if (iSendResult == -1)
    {
        std::cerr << "sendto failed" << std::endl;
    }
}

#endif