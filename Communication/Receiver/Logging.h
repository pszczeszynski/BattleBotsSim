#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "Communication.h"

#define SD_CARD BUILTIN_SDCARD

const int MAX_FILE_SIZE_MB = 1024; // Maximum file size in megabytes

class Logger
{
public:
    Logger(char* fileName);
    String formatDriveCommand(DriveCommand command);
    String formatRobotMessage(RobotMessage robotMessage);
    bool logString(String message);
    bool logMessage(String message);

private:
    bool checkOverflow(File dataFile);

    bool initialized;
    bool overflow;
    char* fileName;
    File dataFile;
};
