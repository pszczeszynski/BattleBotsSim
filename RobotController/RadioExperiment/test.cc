
#include "windows.h"
#include <fstream>
#include <functional>
#include <iostream>
#include <chrono>

#define TRANSMITTER_COM_PORT TEXT("COM7")
#define COM_READ_TIMEOUT_MS 100
#define COM_WRITE_TIMEOUT_MS 100


HANDLE _comPort;


long getCurrTimeMillis()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void _InitComPort()
{
    static int numTries = 0;
    const int NUM_TRIES_BEFORE_PRINTING_ERROR = 5000;

    // close it if it is open
    CloseHandle(_comPort);

    _comPort = CreateFile(TRANSMITTER_COM_PORT, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (_comPort == INVALID_HANDLE_VALUE)
    {
        // increase numTries
        numTries++;
        numTries %= NUM_TRIES_BEFORE_PRINTING_ERROR;

        // if we have tried NUM_TRIES_BEFORE_PRINTING_ERROR times, print error
        if (numTries % NUM_TRIES_BEFORE_PRINTING_ERROR == 1)
        {
            std::cerr << "ERROR: can't open comport" << std::endl;
        }

        // break out of function
        return;
    }
    else
    {
        numTries = 0;
        std::cout << "Transmitter COM Port opened successfully" << std::endl;
    }
    DCB _dcbSerialParams;

    _dcbSerialParams = {0};
    _dcbSerialParams.DCBlength = sizeof(_dcbSerialParams);
    _dcbSerialParams.BaudRate = 460800; // set the baud rate
    _dcbSerialParams.ByteSize = 8;
    _dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    _dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    
    _dcbSerialParams.StopBits = ONESTOPBIT;
    _dcbSerialParams.Parity = NOPARITY;

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = COM_READ_TIMEOUT_MS;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = COM_WRITE_TIMEOUT_MS;


    if (!SetupComm(_comPort, 21 * 3, 21 * 3)) { // Smaller buffer sizes
        std::cerr << "Error setting COM buffer sizes" << std::endl;
    }


    if (!SetCommTimeouts(_comPort, &timeouts))
    {
        std::cerr << "Error setting COM port timeouts" << std::endl;
    }

    if (!SetCommState(_comPort, &_dcbSerialParams))
    {
        std::cerr << "Error setting serial port state" << std::endl;
    }
}


bool readChar(char& c)
{
    DWORD dwErrors = 0;
    COMSTAT comStat;

    // Get and clear current errors on the com port.
    if (!ClearCommError(_comPort, &dwErrors, &comStat))
    {
        // attempt to reinitialize com port
        _InitComPort();
        std::cerr << "Error clearing COM port" << std::endl;

        return false;
    }

    
    DWORD bytesRead;
    ReadFile(_comPort, &c, 1, &bytesRead, NULL);

    if (bytesRead == 0)
    {
        // attempt to reinitialize com port
        _InitComPort();
        std::cerr << "Error reading from COM port" << std::endl;

        return false;
    }
    return true;
}

int main()
{
    static int numTries = 0;
    const int NUM_TRIES_BEFORE_PRINTING_ERROR = 5000;

    _comPort = CreateFile(TRANSMITTER_COM_PORT, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    // close it if it is open
    CloseHandle(_comPort);

    _comPort = CreateFile(TRANSMITTER_COM_PORT, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (_comPort == INVALID_HANDLE_VALUE)
    {
        // increase numTries
        numTries++;
        numTries %= NUM_TRIES_BEFORE_PRINTING_ERROR;

        // if we have tried NUM_TRIES_BEFORE_PRINTING_ERROR times, print error
        if (numTries % NUM_TRIES_BEFORE_PRINTING_ERROR == 1)
        {
            std::cerr << "ERROR: can't open comport" << std::endl;
        }

    }
    else
    {
        numTries = 0;
        std::cout << "Transmitter COM Port opened successfully" << std::endl;
    }
    DCB _dcbSerialParams;

    _dcbSerialParams = {0};
    _dcbSerialParams.DCBlength = sizeof(_dcbSerialParams);
    _dcbSerialParams.BaudRate = 460800; // set the baud rate
    _dcbSerialParams.ByteSize = 8;
    _dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    _dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    
    _dcbSerialParams.StopBits = ONESTOPBIT;
    _dcbSerialParams.Parity = NOPARITY;

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = COM_READ_TIMEOUT_MS;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = COM_WRITE_TIMEOUT_MS;


    // if (!SetupComm(_comPort, sizeof(RobotMessage) * 3, sizeof(RobotMessage) * 3)) { // Smaller buffer sizes
    //     std::cerr << "Error setting COM buffer sizes" << std::endl;
    // }


    if (!SetCommTimeouts(_comPort, &timeouts))
    {
        std::cerr << "Error setting COM port timeouts" << std::endl;
    }

    if (!SetCommState(_comPort, &_dcbSerialParams))
    {
        std::cerr << "Error setting serial port state" << std::endl;
    }

    char last_char = '\0';
    int count = 0;
    while (true)
    {
        char c;
        bool status = readChar(c);


        if (c == '<' && last_char == '>')
        {
            count ++;
            
            int millis_mod_1000 = getCurrTimeMillis() % 1000;
            std::cout << "Time: " << millis_mod_1000 << " count: " << count << std::endl;
            // print time in millis
        }

        last_char = c;

        // print char
        if (!status)
        {
            std::cout << "Error reading char" << std::endl;
            continue;
        }
        std::cout << c;
    }


    return 0;
}