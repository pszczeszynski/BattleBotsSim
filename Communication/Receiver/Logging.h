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
    void updateRadioCommandData(float move, float turn, float bar, float disk);
    void updateVescData(float *fetTemps, float *voltages, float *currents, float *motorTemps, int *erpms, float* dutyCycle);
    void updateIMUData(float rotations, float rotationVelocitys, float accelX, float accelY);
    void updateRadioData(int radioStatus, int powerLevel, long invalidPacketCount, int radioChannel);
    void updateSelfRighterData(float speed);
    void updateFlag(bool flag);
    void update();
    bool init();

private:
    const String escs[ESC_COUNT] = {"LD", "RD", "FW", "RW"};
    const String axis[AXIS_COUNT] = {"X", "Y", "Z"};
    const String inputs[ESC_COUNT] = {"Move", "Turn", "Bar", "Disk"};
    bool checkOverflow(File dataFile);

    int highestLog = 1;

    long long lastLogTime = 0;
    
    float radioCommands[ESC_COUNT];

    float dutyCycle[ESC_COUNT];
    float fetTemps[ESC_COUNT];
    float voltages[ESC_COUNT];
    float currents[ESC_COUNT];
    float motorTemps[ESC_COUNT];
    int erpms[ESC_COUNT];

    float rotations[AXIS_COUNT];
    float rotationVelocitys[AXIS_COUNT];
    float accels[AXIS_COUNT];

    int radioStatus;
    int powerLevel;
    long invalidPacketCount;
    int radioChannel;

    float selfRighterSpeed;

    bool noteFlag;

    bool initialized;
    bool overflow;
    char* fileName;
    File dataFile;
};
