#include "Logging.h"

String DoubleToString(double value)
{
  char buffer[15]; // Adjust the buffer size as needed

  dtostrf(value, 8, 3, buffer); // Convert to string with 4 decimal places

  return String(buffer);
}


Logger::Logger(char* fileName = "dataLog.txt")
{
    this->fileName = fileName;
    overflow = false;

    // Initialize SD card
    initialized = true;
    if (!SD.begin(SD_CARD)) 
    {
        Serial.println("SD initialization failed.");
        initialized = false;
    }

    if (initialized) 
    {  
        // Clear logs
        if (SD.exists(fileName)) SD.remove(fileName);

        logMessage("Beginning Match Log. Good Luck!\n");
    }
}

/**
  * Formats drive command into a nice string
  */
String Logger::formatDriveCommand(DriveCommand command)
{
    String message = "";
    message += DoubleToString(millis() / 1000.0);
    message += ": DRIVE COMMAND{ M : ";
    message += DoubleToString(command.movement);
    message += "; T : ";
    message += DoubleToString(command.turn);
    message += "; FW : ";
    message += DoubleToString(command.frontWeaponPower);
    message += "; RW : ";
    message += DoubleToString(command.backWeaponPower);
    message += " }\n";

    return message;
}


/**
  * Formats robot message into a nice string, split into IMU data and VESC data
  */
String Logger::formatRobotMessage(RobotMessage robotMessage)
{
    // IMU Portion
    String message = "";
    message += DoubleToString(millis() / 1000.0);
    message += ": IMU DATA     { ROT : ";
    message += DoubleToString(robotMessage.rotation);
    message += "; ROT_VEL : ";
    message += DoubleToString(robotMessage.rotationVelocity);
    message += "; ACCEL_X : ";
    message += DoubleToString(robotMessage.accelX);
    message += "; ACCEL_Y : ";
    message += DoubleToString(robotMessage.accelY);
    message += " }\n";

    // ESC Data
    String names[4] = {"LEFT DRIVE  ", "RIGHT DRIVE ", "FRONT WEAPON", "REAR WEAPON "};
    for (int i = 0; i < 4; i++)
    {
        message += DoubleToString(millis() / 1000.0);
        message += ": ";
        message += names[i];
        message += " { I : ";
        message += DoubleToString(robotMessage.motorCurrent[i]);
        message += "; V : ";
        message += DoubleToString(robotMessage.motorVoltage[i]);
        message += "; RPM : ";
        message += DoubleToString(robotMessage.motorRPM[i]);
        message += "; TEMP : ";
        message += DoubleToString(robotMessage.escFETTemp[i]);
        message += " }\n";
    }

    return message;
}

/**
  * Writes given message to datalog.txt on the SD card
  */
bool Logger::logMessage(String message)
{
    Serial.println("Begin Log");
    if (initialized && !overflow)
    {
        // open the file.
        dataFile = SD.open(fileName, FILE_WRITE);
        // if the file is available, write to it:
        if (dataFile) {
            dataFile.println(message);
            checkOverflow(dataFile);
            dataFile.close();
            Serial.println("End Log");
            return true;
        } else {
            // if the file isn't open, pop up an error:
            Serial.println("Error opening datalog.txt");
            Serial.println("End Log");
            return false;
        }
    }

    if (overflow) Serial.println("Error: File size overflowed. Unable to write to file.");
    else Serial.println("Error: SD card not initialized. Unable to write to file.");
    Serial.println("End Log");
    return false;
}


/**
  * Checks if file size exceeds current limit
  */
bool Logger::checkOverflow(File dataFile)
{
  Serial.println(dataFile.size());
  if (dataFile.size() >= MAX_FILE_SIZE_MB * 1024 * 1024)
  {
    Serial.println("Error: File size exceeded set limit.");
    overflow = true;
    return true;
  }

  return false;
}