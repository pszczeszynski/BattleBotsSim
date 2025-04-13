#pragma once

#include "../Common/Communication.h"
#include <fstream>
#include <filesystem>
#include <string>
#include <iostream>


struct DSMessageLog
{
    uint32_t timestamp;
    DriverStationMessage msg;
};

struct RobotMessageLog
{
    uint32_t timestamp;
    RobotMessage msg;
};

class DriverStationLog
{
    public:
        DriverStationLog();
        void UpdateTxLog(uint32_t timestamp, DriverStationMessage command);
        void UpdateRxLog(uint32_t timestamp, RobotMessage message);
        void CloseLog();
        std::string GetLogDirectory();

    private:
        std::ofstream _outFileTx;
        std::ofstream _outFileBinTx;
        std::ofstream _outFileRx;
        std::ofstream _outFileBinRx;

        std::string _logDirectory;
        
        std::string _log_header = "RADIO LOG CSV V1";
};