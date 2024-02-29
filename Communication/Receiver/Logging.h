#include "WString.h"
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "Communication.h"

#define SD_CARD 10
#define LOG_TIME 1/25 //value in hz
#define ESC_COUNT 4
#define AXIS_COUNT 3
#define LOG_PARAM_COUNT 40

const int MAX_FILE_SIZE_MB = 1024; // Maximum file size in megabytes

class Logger
{
public:
    Logger();
    String formatDriveCommand(DriveCommand command);
    String formatRobotMessage(RobotMessage robotMessage);
    bool logMessage(String message);
    void updateCommandData(double ld, double rd, double fw, double rw);
    void updateVescData(double *fetTemps, double *voltages, double *currents, double *motorTemps, double *erpms);
    void updateIMUData(double *rotations, double *rotationVelocitys, double *accels);
    void updateRadioData(int radioStatus, int powerLevel, long inavalidPacketCount, int radioChannel);
    void updateSelfRighterData(int microseconds);

    void update();

private:
    String escs[ESC_COUNT] = {"LD", "RD", "FW", "RW"};
    String axis[AXIS_COUNT] = {"X", "Y", "Z"};
    bool checkOverflow(File dataFile);

    int highestLog = 1;

    long long lastLogTime = 0;

    double commands[ESC_COUNT];
    double fetTemps[ESC_COUNT];
    double voltages[ESC_COUNT];
    double currents[ESC_COUNT];
    double motorTemps[ESC_COUNT];
    double erpms[ESC_COUNT];

    double rotations[AXIS_COUNT];
    double rotationVelocitys[AXIS_COUNT];
    double accels[AXIS_COUNT];

    int radioStatus;
    int powerLevel;
    long inavalidPacketCount;
    int radioChannel;

    int selfRighterMicros;

    bool noteFlag;

    bool initialized;
    bool overflow;
    char* fileName;
    File dataFile;
};
