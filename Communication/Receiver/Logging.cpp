#include "Logging.h"

String DoubleToString(double value) {
  char buffer[15];  // Adjust the buffer size as needed

  dtostrf(value, 8, 3, buffer);  // Convert to string with 4 decimal places

  return String(buffer);
}

  
Logger::Logger() {
  overflow = false;

  // Initialize SD card
  initialized = true;
  if (!SD.begin(SD_CARD)) {
    Serial.println("SD initialization failed.");
    initialized = false;
  }

  File root = SD.open("/");

  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      //Serial.println("** no more files **");
      break;
    }
    if (!entry.isDirectory()) {
      // is a file
      String name = String(entry.name());
      //figure out what the highest previous log is, then incrament
      if (name.startsWith("log")) {
        name.remove(0, 3);
        int length = name.length();
        name.remove(length - 4, 4);
        int num = name.toInt();
        if (num >= highestLog) {
          highestLog = num;
        }
      }
    }
    entry.close();
  }

  char buf[15];

  sprintf(buf, "log%d.csv", highestLog++);

  this->fileName = buf;

  dataFile = SD.open(fileName, FILE_WRITE);
  String header = "";
  header += "TIME,";
  for (int i = 0; i<=ESC_COUNT; i++){
    header += (escs[i] + "COMMAND,");
    header += (escs[i] + "FET_TEMP,");
    header += (escs[i] + "VOTLAGE,");
    header += (escs[i] + "CURRENT,");
    header += (escs[i] + "MOTOR_TEMP,");
    header += (escs[i] + "ERPM,");
  }

  for (int i = 0; i<=AXIS_COUNT; i++){
    header += (axis[i] + "ROTATION,");
    header += (axis[i] + "ROTATION_VELOCITY,");
    header += (axis[i] + "ACCEL,");
  }
  header += "RADIO_STATUS,";
  header += "POWER_LEVEL,";
  header += "INVALID_PACKETS,";

  
  header += "FLAG";

  dataFile.println(header);
}

void Logger::update(){
  

}

void Logger::updateCommandData(double ld, double rd, double fw, double rw){
  commands[0] = ld;
  commands[1] = rd;
  commands[2] = fw;
  commands[3] = rw;

}

void Logger::updateVescData(double *fetTemps, double *voltages, double *currents, double *motorTemps, double *erpms){
  memcpy(this->fetTemps, fetTemps, sizeof(double)*4);
  memcpy(this->voltages, voltages, sizeof(double)*4);
  memcpy(this->currents, currents, sizeof(double)*4);
  memcpy(this->motorTemps, motorTemps, sizeof(double)*4);
  memcpy(this->ermps, ermps, sizeof(double)*4);

}

void Logger::updateIMUData(double *rotations, double *rotationVelocitys, double *accels){
  memcpy(this->rotations, rotations, sizeof(double)*4);
  memcpy(this->rotationVelocitys, rotationVelocitys, sizeof(double)*4);
  memcpy(this->accels, accels, sizeof(double)*4);
}

void Logger::updateRadioData(){

}



/**
  * Formats robot message into a nice string, split into IMU data and VESC data
  */
String Logger::formatRobotMessage(RobotMessage robotMessage) {
  // IMU Portion
  String message = "";

  // if the message is CAN data
  if (robotMessage.type == CAN_DATA) {
    String names[4] = { "LEFT DRIVE  ", "RIGHT DRIVE ", "FRONT WEAPON", "REAR WEAPON " };
    for (int i = 0; i < 4; i++) {
      message += DoubleToString(millis() / 1000.0);
      message += ": ";
      message += names[i];
      message += " { I : ";
      message += DoubleToString(robotMessage.canData.motorCurrent[i]);
      message += "; V : ";
      message += DoubleToString(robotMessage.canData.motorVoltage[i]);
      message += "; ERPM : ";
      message += DoubleToString(robotMessage.canData.motorERPM[i]);
      message += "; TEMP : ";
      message += DoubleToString(robotMessage.canData.escFETTemp[i]);
      message += " }\n";
    }
  }
  // else if the message is IMU data
  else if (robotMessage.type == IMU_DATA) {
    message += DoubleToString(millis() / 1000.0);
    message += ": IMU DATA     { ROT : ";
    message += DoubleToString(robotMessage.imuData.rotation);
    message += "; ROT_VEL : ";
    message += DoubleToString(robotMessage.imuData.rotationVelocity);
    message += "; ACCEL_X : ";
    message += DoubleToString(robotMessage.imuData.accelX);
    message += "; ACCEL_Y : ";
    message += DoubleToString(robotMessage.imuData.accelY);
    message += " }\n";
  }
  // else invalid message
  else {
    message += DoubleToString(millis() / 1000.0);
    message += ": INVALID MESSAGE!\n";
  }

  return message;
}

/**
  * Writes given message to datalog.txt on the SD card
  */
bool Logger::logMessage(String message) {
  if (initialized && !overflow) {
    // open the file.
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(message);
      checkOverflow(dataFile);
      dataFile.close();
      // Serial.println("End Log");
      return true;
    } else {
      // if the file isn't open, pop up an error:
      // Serial.println("Error opening datalog.txt");
      // Serial.println("End Log");
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
bool Logger::checkOverflow(File dataFile) {
  Serial.println(dataFile.size());
  if (dataFile.size() >= MAX_FILE_SIZE_MB * 1024 * 1024) {
    Serial.println("Error: File size exceeded set limit.");
    overflow = true;
    return true;
  }

  return false;
}