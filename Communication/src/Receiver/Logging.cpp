#include "Receiver/Logging.h"

Logger::Logger() {
  Serial.println("Initializing SD card...");
  // zero out all the data
  for (int i = 0; i < ESC_COUNT; i++) {
    radioCommands[i] = 0;
    dutyCycle[i] = 0;
    fetTemps[i] = 0;
    voltages[i] = 0;
    currents[i] = 0;
    motorTemps[i] = 0;
    erpms[i] = 0;
  }

  // zero out rotations and accels
  for (int i = 0; i < AXIS_COUNT; i++) {
    rotations[i] = 0;
    rotationVelocitys[i] = 0;
    accels[i] = 0;
  }

  // zero out radio data
  // radioStatus = 0;
  // powerLevel = 0;
  // invalidPacketCount = 0;
  // radioChannel = 0;

  // init overflow
  overflow = false;
}

bool Logger::init() {
  // check if sd card exists
  initialized = SD.begin(SD_CARD);
  if (!initialized) {
    // return failure
    return false;
  }

  File root = SD.open("/");

  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      // Serial.println("** no more files **");
      break;
    }
    if (!entry.isDirectory()) {
      // is a file
      String name = String(entry.name());
      // figure out what the highest previous log is, then incrament
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

  highestLog++;

  printf("highest log: %d \n", highestLog);

  char buf[15];

  sprintf(buf, "log%d.csv", highestLog);

  this->fileName = buf;

  dataFile = SD.open(fileName, FILE_WRITE);
  printf("made new file: %s\n", fileName);
  String header = "";
  header += "TIME,";
  for (int i = 0; i < ESC_COUNT; i++) {
    header += (inputs[i]) + ",";
  }
  for (int i = 0; i < ESC_COUNT; i++) {
    header += (escs[i] + "_DUTY_CYCLE,");
    header += (escs[i] + "_FET_TEMP,");
    header += (escs[i] + "_VOTLAGE,");
    header += (escs[i] + "_CURRENT,");
    header += (escs[i] + "_MOTOR_TEMP,");
    header += (escs[i] + "_ERPM,");
  }

  for (int i = 0; i < AXIS_COUNT; i++) {
    header += (axis[i] + "_ROTATION,");
    header += (axis[i] + "_ROTATION_VELOCITY,");
    header += (axis[i] + "_ACCEL,");
  }
  header += "RADIO_STATUS,";
  header += "POWER_LEVEL,";
  header += "INVALID_PACKETS,";
  header += "RADIO_CHANNEL,";

  header += "SELF_RIGHTER,";

  header += "FLAG";

  dataFile.println(header);

  dataFile.flush();

  Serial.println("Success!");
  return true;
}

void Logger::update() {
  if (!initialized) {
    return;
  }

  if (millis() - lastLogTime >= LOG_TIME) {
    // time to log
    String logMsg;

    logMsg += millis();
    logMsg += ",";

    for (int i = 0; i < ESC_COUNT; i++) {
      logMsg += (String)radioCommands[i];
      logMsg += ",";
    }

    for (int i = 0; i < ESC_COUNT; i++) {
      logMsg += (String)dutyCycle[i];
      logMsg += ",";
      logMsg += (String)fetTemps[i];
      logMsg += ",";
      logMsg += (String)voltages[i];
      logMsg += ",";
      logMsg += (String)currents[i];
      logMsg += ",";
      logMsg += (String)motorTemps[i];
      logMsg += ",";
      logMsg += (String)erpms[i];
      logMsg += ",";
    }

    for (int i = 0; i < AXIS_COUNT; i++) {
      logMsg += (String)rotations[i];
      logMsg += ",";
      logMsg += (String)rotationVelocitys[i];
      logMsg += ",";
      logMsg += (String)accels[i];
      logMsg += ",";
    }
    logMsg += (String)radioStatus;
    logMsg += ",";
    logMsg += (String)powerLevel;
    logMsg += ",";
    logMsg += (String)invalidPacketCount;
    logMsg += ",";
    logMsg += (String)radioChannel;
    logMsg += ",";

    logMsg += (String)selfRighterSpeed;
    logMsg += ",";

    logMsg += (String)noteFlag;

    dataFile.println(logMsg);
    dataFile.flush();
  }
}

void Logger::updateRadioCommandData(float move, float turn, float bar,
                                    float disk) {
  radioCommands[0] = move;
  radioCommands[1] = turn;
  radioCommands[2] = bar;
  radioCommands[3] = disk;
}

void Logger::updateVescData(float *fetTemps, float *voltages, float *currents,
                            float *motorTemps, int *erpms, float *dutyCycle) {
  memcpy(this->fetTemps, fetTemps, sizeof(float) * 4);
  memcpy(this->voltages, voltages, sizeof(float) * 4);
  memcpy(this->currents, currents, sizeof(float) * 4);
  memcpy(this->motorTemps, motorTemps, sizeof(float) * 4);
  memcpy(this->erpms, erpms, sizeof(int) * 4);
  memcpy(this->dutyCycle, dutyCycle, sizeof(float) * 4);
}

void Logger::updateIMUData(float rotations, float rotationVelocitys,
                           float accelX, float accelY) {
  this->rotations[0] = rotations;
  this->rotationVelocitys[0] = rotationVelocitys;
  this->accels[0] = accelX;
  this->accels[1] = accelY;
}

void Logger::updateRadioData(int radioStatus, int powerLevel,
                             long invalidPacketCount, int radioChannel) {
  this->radioStatus = radioStatus;
  this->powerLevel = powerLevel;
  this->invalidPacketCount = invalidPacketCount;
  this->radioChannel = radioChannel;
}

void Logger::updateSelfRighterData(float speed) {
  this->selfRighterSpeed = speed;
}

void Logger::updateFlag(bool flag) { this->noteFlag = flag; }

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