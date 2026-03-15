#include "IRobotLink.h"

#include "../../Common/Communication.h"
#include "../Clock.h"
#include "../UIWidgets/GraphWidget.h"

RobotMessage IRobotLink::Receive() {
  static GraphWidget recvIntervalTime("Received Packet Interval Time", 0, 50,
                                      "ms");

  std::vector<RobotMessage> newMessages = _ReceiveImpl();

  if (newMessages.size() == 0) {
    return RobotMessage{RobotMessageType::INVALID};
  }

  bool hadValidMessage = false;

  int messageIndex = 0;
  for (RobotMessage &msg : newMessages) {
    if (msg.type == RobotMessageType::IMU_DATA) {
      _lastIMUMessageMutex.lock();
      _lastIMUMessage = msg;
      _lastIMUMessageMutex.unlock();

      if (_imuCallback && messageIndex < (int)_lastMessageReceiveTimes.size()) {
        _imuCallback(msg.imuData, _lastMessageReceiveTimes[messageIndex]);
      }

      hadValidMessage = true;
    } else if (msg.type == RobotMessageType::IMU_DEBUG_DATA) {
      _lastIMUDebugMessageMutex.lock();
      _lastIMUDebugMessage = msg;
      _lastIMUDebugMessageMutex.unlock();
      hadValidMessage = true;
    } else if (msg.type == RobotMessageType::CAN_DATA) {
      _lastCANMessageMutex.lock();
      _lastCANMessage = msg;
      _lastCANMessageMutex.unlock();
      hadValidMessage = true;
    } else if (msg.type == RobotMessageType::RADIO_DATA) {
      _lastRadioMessageMutex.lock();
      _lastRadioMessage = msg;
      _lastRadioMessageMutex.unlock();
      hadValidMessage = true;
    } else if (msg.type == RobotMessageType::BOARD_TELEMETRY_DATA) {
      _lastBoardTelemetryMessageMutex.lock();
      _lastBoardTelemetryMessage = msg;
      _lastBoardTelemetryMessageMutex.unlock();
      hadValidMessage = true;
    }
    messageIndex++;
  }

  if (!hadValidMessage) {
    std::cerr << "ERROR: invalid message type" << std::endl;
    return RobotMessage{RobotMessageType::INVALID};
  }

  for (RobotMessage &msg : newMessages) {
    recvIntervalTime.AddData(_receiveClock.getElapsedTime() * 1000);
    _receiveClock.markStart();
  }

  return newMessages.back();
}

RobotMessage IRobotLink::GetLastCANMessage() {
  RobotMessage ret;
  _lastCANMessageMutex.lock();
  ret = _lastCANMessage;
  _lastCANMessageMutex.unlock();
  return ret;
}

RobotMessage IRobotLink::GetLastRadioMessage() {
  RobotMessage ret;

  if (_receiveClock.getElapsedTime() * 1000 > 100) {
    memset(&ret, 0, sizeof(RobotMessage));
    ret.type = RobotMessageType::RADIO_DATA;
    ret.radioData.averageDelayMS = -1;
    return ret;
  }

  _lastRadioMessageMutex.lock();
  ret = _lastRadioMessage;
  _lastRadioMessageMutex.unlock();
  return ret;
}

RobotMessage IRobotLink::GetLastBoardTelemetryMessage() {
  RobotMessage ret;
  _lastBoardTelemetryMessageMutex.lock();
  ret = _lastBoardTelemetryMessage;
  _lastBoardTelemetryMessageMutex.unlock();
  return ret;
}

RobotMessage IRobotLink::GetLastIMUDebugMessage() {
  RobotMessage ret;
  _lastIMUDebugMessageMutex.lock();
  ret = _lastIMUDebugMessage;
  _lastIMUDebugMessageMutex.unlock();
  return ret;
}

bool IRobotLink::IsTransmitterConnected() { return _transmitterConnected; }

bool IRobotLink::IsSecondaryTransmitterConnected() {
  return _secondaryTransmitterConnected;
}

void IRobotLink::RegisterIMUCallback(IMUCallback callback) {
  _imuCallback = std::move(callback);
}
