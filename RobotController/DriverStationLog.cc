#include "DriverStationLog.h"

const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y_%m_%d_%H_%M_%S", &tstruct);
    return buf;
}

DriverStationLog::DriverStationLog()
{
    _logDirectory = "./Logs/" + currentDateTime();
    std::filesystem::create_directories(_logDirectory);

    std::string logFileTx = _logDirectory + "/radio_sent.log";
    std::string logFileBinTx = _logDirectory + "/radio_sent.bin";
    _outFileTx.open(logFileTx);
    _outFileBinTx.open(logFileBinTx);
    _outFileTx << _log_header << std::endl;
    _outFileBinTx << _log_header << std::endl;

    std::string logFileRx = _logDirectory + "/radio_recv.log";
    std::string logFileBinRx = _logDirectory + "/radio_recv.bin";
    _outFileRx.open(logFileRx);
    _outFileBinRx.open(logFileBinRx);
    _outFileRx << _log_header << std::endl;
    _outFileBinRx << _log_header << std::endl;
}

void DriverStationLog::UpdateTxLog(uint32_t timestamp, DriverStationMessage command)
{
    // store the message as raw bytes into a .bin file
    DSMessageLog log;
    log.msg = command;
    log.timestamp = timestamp;
    _outFileBinTx.write(reinterpret_cast<const char*>(&log), sizeof(log));

    // store the message as a human-readable (ish) csv
    std::string line = std::to_string(command.type);
    line += "," + std::to_string(command.timestamp);

    if (command.type == DRIVE_COMMAND)
    {
        line += "," + std::to_string(command.driveCommand.movement);
        line += "," + std::to_string(command.driveCommand.turn);
        line += "," + std::to_string(command.driveCommand.frontWeaponPower);
        line += "," + std::to_string(command.driveCommand.backWeaponPower);
        line += "," + std::to_string(command.driveCommand.selfRighterPower);
        line += "," + std::to_string(command.driveCommand.selfRighterDuty);
    }
    else if (command.type == AUTO_DRIVE)
    {
        line += "," + std::to_string(command.autoDrive.movement);
        line += "," + std::to_string(command.autoDrive.targetAngle);
        line += "," + std::to_string(command.autoDrive.targetAngleVelocity);
        line += "," + std::to_string(command.autoDrive.KD_PERCENT);
        line += "," + std::to_string(command.autoDrive.TURN_THRESH_1_DEG);
        line += "," + std::to_string(command.autoDrive.TURN_THRESH_2_DEG);
        line += "," + std::to_string(command.autoDrive.MAX_TURN_POWER_PERCENT);
        line += "," + std::to_string(command.autoDrive.MIN_TURN_POWER_PERCENT);
        line += "," + std::to_string(command.autoDrive.SCALE_DOWN_MOVEMENT_PERCENT);
        line += "," + std::to_string(command.autoDrive.invertTurn);
        line += "," + std::to_string(command.autoDrive.frontWeaponCurrent10);
        line += "," + std::to_string(command.autoDrive.backWeaponCurrent10);
    }
    
    line += "," + std::to_string(command.resetIMU);
    line += "," + std::to_string(command.fuseIMU);
    line += "," + std::to_string(command.valid);

    _outFileTx << line << std::endl;
    _outFileTx.flush();  // Flush stream buffer
    _outFileTx.sync_with_stdio();  // Forces OS to write all buffered data to disk
    _outFileBinTx.flush();
    _outFileBinTx.sync_with_stdio();
}

void DriverStationLog::UpdateRxLog(uint32_t timestamp, RobotMessage message)
{
    RobotMessageLog log;
    log.msg = message;
    log.timestamp = timestamp;
    _outFileBinRx.write(reinterpret_cast<const char*>(&log), sizeof(log));

    std::string line = std::to_string(message.type);
    line += "," + std::to_string(message.timestamp);
    if (message.type == IMU_DATA)
    {
        line += "," + std::to_string(message.imuData.rotation);
        line += "," + std::to_string(message.imuData.rotationVelocity);
        line += "," + std::to_string(message.imuData.accelX);
        line += "," + std::to_string(message.imuData.accelY);
    }
    else if (message.type == CAN_DATA)
    {
        for(int i = 0; i < 5; i++)
        {
            line += "," + std::to_string(message.canData.motorCurrent[i]);
            line += "," + std::to_string(message.canData.motorVoltage[i]);
            line += "," + std::to_string(message.canData.motorERPM[i]);
            line += "," + std::to_string(message.canData.escFETTemp[i]);
            line += "," + std::to_string(message.canData.motorTemp[i]);
        }
    }
    else if (message.type == RADIO_DATA)
    {
        line += "," + std::to_string(message.radioData.averageDelayMS);
        line += "," + std::to_string(message.radioData.maxDelayMS);
        line += "," + std::to_string(message.radioData.movement);
        line += "," + std::to_string(message.radioData.turn);
        line += "," + std::to_string(message.radioData.frontWeaponPower);
        line += "," + std::to_string(message.radioData.backWeaponPower);
        line += "," + std::to_string(message.radioData.invalidPackets);
    }
    else if (message.type == BOARD_TELEMETRY_DATA)
    {
        line += "," + std::to_string(message.boardTelemetryData.voltage_batt);
        line += "," + std::to_string(message.boardTelemetryData.voltage_5v);
        line += "," + std::to_string(message.boardTelemetryData.current_5v);
        line += "," + std::to_string(message.boardTelemetryData.voltage_3v3);
        line += "," + std::to_string(message.boardTelemetryData.temperature);
    }
    line += "," + std::to_string(message.valid);

    _outFileRx << line << std::endl;
    _outFileBinRx.flush();
    _outFileBinRx.sync_with_stdio();
    _outFileRx.flush();  // Flush stream buffer
    _outFileRx.sync_with_stdio();  // Forces OS to write all buffered data to disk
}

std::string DriverStationLog::GetLogDirectory()
{
    return _logDirectory;
}

void DriverStationLog::CloseLog()
{
    _outFileTx.close();
    _outFileRx.close();
}