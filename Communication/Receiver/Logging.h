#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "Communication.h"

#define SD_CARD 10
#define LOG_TIME 1/25 //value in hz

const int MAX_FILE_SIZE_MB = 1024; // Maximum file size in megabytes

class Logger
{
public:
    Logger();
    String formatDriveCommand(DriveCommand command);
    String formatRobotMessage(RobotMessage robotMessage);
    bool logMessage(String message);
    void updateCommandData(double ld, double rd, double fw, double rw);
    void updateVescData(double *fetTemps, double *voltages, double *current, double *motorTemps, double *erpms);
    void updateIMUData(double *rotations, double *rotationVelocitys, double *accels);

    void update();

private:
    bool checkOverflow(File dataFile);

    int highestLog = 0;

    long long lastLogTime = 0;

    double commands[4];
    double fetTemps[4];
    double voltages[4];
    double currents[4];
    double motorTemps[4];
    double ermps[4];
    double rotations[3];
    double rotationVelocitys[3];
    double accels[3];

    bool initialized;
    bool overflow;
    char* fileName;
    File dataFile;
};
