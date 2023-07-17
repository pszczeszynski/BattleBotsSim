#include "RobotLink.h"

const char *TEENSY_PORT_NAME = "COM4";

RobotLinkReal::RobotLinkReal()
{
    LPCSTR portNameLPCSTR = TEENSY_PORT_NAME;

    // Open the port
    HANDLE hPort = CreateFile((LPCWSTR) portNameLPCSTR, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    if (hPort == INVALID_HANDLE_VALUE)
    {
        printf("Error opening port\n");
        return;
    }

    // Set up the port settings
    DCB dcb;
    memset(&dcb, 0, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    dcb.BaudRate = CBR_9600;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;

    if (!SetCommState(hPort, &dcb))
    {
        printf("Error setting port state\n");
        CloseHandle(hPort);
        return;
    }
}

void RobotLinkReal::Drive(DriveCommand& command)
{
    // 1. serialize
    std::string byteString(reinterpret_cast<const char*>(&command), sizeof(DriveCommand));
    const char* serialized_cstr = byteString.c_str();
    
    // 2. send message over radio
    DWORD bytesWritten;
    if (!WriteFile(hPort, serialized_cstr, strlen(serialized_cstr), &bytesWritten, NULL))
    {
        printf("Error writing to port\n");
        return;
    }
}

#define MAX_BUFFER_LENGTH 2048
RobotMessage RobotLinkReal::Receive()
{
    // Read the response
    char buffer[MAX_BUFFER_LENGTH];
    DWORD bytesRead;
    if (!ReadFile(hPort, buffer, sizeof(buffer), &bytesRead, NULL))
    {
        printf("Error reading from port\n");
    }
    else
    {
        buffer[bytesRead] = '\0'; // Null terminate the string
        printf("Received: %s\n", buffer);
    }

    // Retrieve the struct from the string
    RobotMessage retrievedStruct;
    std::memcpy(&retrievedStruct, buffer, sizeof(retrievedStruct));

    return retrievedStruct;
}

RobotLinkReal::~RobotLinkReal()
{
    // Close the port
    CloseHandle(hPort);
}


/////////////////// SIMULATION //////////////////////

RobotLinkSim::RobotLinkSim()
    : serverSocket("11115")
{
}

void RobotLinkSim::Drive(DriveCommand& command)
{
    RobotControllerMessage message = { command.movement, command.turn };
    serverSocket.reply_to_last_sender(RobotStateParser::serialize(message));
}

RobotMessage RobotLinkSim::Receive()
{
    std::string received = "";
    while (received == "")
    {
        received = serverSocket.receive();
    }
    RobotState message = RobotStateParser::parse(received);
    
    RobotMessage ret {0};
    ret.accel = Point { message.robot_velocity.x, message.robot_velocity.y, message.robot_velocity.z };
    ret.rotation = message.robot_orientation;

    // return the message
    return ret;
}