#include <windows.h>
#include <stdio.h>

int main()
{
    // Adjust as needed
    const char *portName = "COM4";
    const char *message = "Hello, Teensy!";

    LPCSTR portNameLPCSTR = portName;

    // Open the port
    HANDLE hPort = CreateFile((LPCSTR) portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    if (hPort == INVALID_HANDLE_VALUE)
    {
        printf("Error opening port\n");
        return 1;
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
        return 1;
    }

    // Send the message
    DWORD bytesWritten;
    if (!WriteFile(hPort, message, strlen(message), &bytesWritten, NULL))
    {
        printf("Error writing to port\n");
        CloseHandle(hPort);
        return 1;
    }

    // Read the response
    char buffer[1024];
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

    // Close the port
    CloseHandle(hPort);

    return 0;
}