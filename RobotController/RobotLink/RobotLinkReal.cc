#include "RobotLinkReal.h"

#ifndef SIMULATION

#include "../../Common/Communication.h"
#include "../Clock.h"
#include "../Globals.h"
#include "../MathUtils.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "../UIWidgets/ClockWidget.h"
#include "../UIWidgets/GraphWidget.h"

#define USB_RETRY_TIME 50
#define HID_BUFFER_SIZE 64
#define SEND_TIMEOUT_MS \
  1  // after this time, give up sending and go back to receiving
#define RECEIVE_TIMEOUT_MS \
  100  // Was 5 but doubly defined to 100 later in code, assuming 100 is the
       // intended duration

#define PACKET_LOSS_SWITCH_TIMEOUT_MS 100

#define RADIO_DEVICES 5

#define PRIMARY_TIMEOUT 0.05
void RobotLinkReal::TryConnection(void) {
  _transmitterConnected = false;
  _secondaryTransmitterConnected = false;
  DriverStationMessage ping_request;
  ping_request.type = LOCAL_PING_REQUEST;
  ping_request.valid = true;
  _primary_radio_index = -1;
  _secondary_radio_index = -1;
  char pingbuf[HID_BUFFER_SIZE] = {0};
  char recvbuf[HID_BUFFER_SIZE] = {0};
  memcpy(pingbuf, &ping_request, sizeof(DriverStationMessage));

  Clock c_retryTime;

  while (true) {
    int devices_opened =
        rawhid_open(RADIO_DEVICES, 0x16C0, 0x0486, 0xFFAB, 0x0200);
    c_retryTime.markStart();
    if (devices_opened >= 1) {
#ifdef PINGPONG
      for (int i = 0; i < devices_opened; i++) {
        rawhid_send(i, pingbuf, HID_BUFFER_SIZE, SEND_TIMEOUT_MS);
      }
      while (c_retryTime.getElapsedTime() < PRIMARY_TIMEOUT) {
        for (int i = 0; i < devices_opened; i++) {
          int num = rawhid_recv(i, recvbuf, HID_BUFFER_SIZE, 2);
          if (num >= sizeof(RobotMessage)) {
            RobotMessage msg = *reinterpret_cast<RobotMessage *>(recvbuf);
            if (msg.type == LOCAL_PING_RESPONSE) {
              if (!_transmitterConnected) {
                _transmitterConnected = true;
                _primary_radio_index = i;
              } else if (!_secondaryTransmitterConnected) {
                _secondaryTransmitterConnected = true;
                _secondary_radio_index = i;
                return;
              }
            }
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      if (_transmitterConnected) {
        return;
      }
#else
      if (devices_opened == 1) {
        _transmitterConnected = true;
        _primary_radio_index = 0;
      } else {
        _transmitterConnected = true;
        _primary_radio_index = 0;

        _secondaryTransmitterConnected = true;
        _secondary_radio_index = 1;
      }
      return;
#endif
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(USB_RETRY_TIME));
  }
}

#ifdef PINGPONG
#define SECONDARY_RETRY 5
#else
#define SECONDARY_RETRY 0.5
#endif

void RobotLinkReal::RadioThreadSendFunction(RawHID *dev, bool *newMessage,
                                            DriverStationMessage *message,
                                            std::mutex *messageMutex,
                                            bool delay) {
  int num;
  char buf[HID_BUFFER_SIZE];

  auto start = std::chrono::high_resolution_clock::now();

  while (true) {
    if (!_radio_reinit && dev->IsOpen()) {
      if (!dev->IsSendPending() && *newMessage) {
        if (messageMutex->try_lock()) {
          memcpy(buf, message, sizeof(DriverStationMessage));
          messageMutex->unlock();
          if (delay) {
            auto elapsed =
                std::chrono::high_resolution_clock::now() - _startTime;
            uint32_t start =
                std::chrono::duration_cast<std::chrono::microseconds>(elapsed)
                    .count();
            while (
                std::chrono::duration_cast<std::chrono::microseconds>(elapsed)
                        .count() -
                    start <
                300) {
              elapsed = std::chrono::high_resolution_clock::now() - _startTime;
            }
          }
          if (!dev->SendAsync(buf, HID_BUFFER_SIZE)) _radio_reinit = true;
          *newMessage = false;
        }
      } else if (dev->IsSendPending()) {
        num = dev->CheckSendAsync();
        if (num < 0) {
          _radio_reinit = true;
        }
      }
    }
  }
}

void RobotLinkReal::RadioThreadRecvFunction(
    RawHID *dev, std::mutex *messageMutex,
    std::deque<RobotMessage> *messageQueue, RobotMessage *radioStats,
    Clock *lastReceivedTimer, std::mutex *radioStatsMutex) {
  static GraphWidget packetRoundTrip("Packet Round-trip Time", 0, 5, "ms",
                                     1000);
  int num;
  char buf[HID_BUFFER_SIZE];

  Clock recvTimeout;

  while (true) {
    if (!_radio_reinit && dev->IsOpen()) {
      if (!dev->IsRecvPending()) {
        if (!dev->RecvAsync(HID_BUFFER_SIZE)) _radio_reinit = true;
        recvTimeout.markStart();
      } else {
        num = dev->CheckRecvAsync(HID_BUFFER_SIZE, buf);
        if (num < 0) _radio_reinit = true;
        if (num >= sizeof(RobotMessage)) {
          RobotMessage msg = *reinterpret_cast<RobotMessage *>(buf);

          if (msg.type != RobotMessageType::INVALID) {
            _outstandingPacketMutex.lock();
            if (_outstandingPackets.find(msg.timestamp) !=
                _outstandingPackets.end()) {
              _outstandingPackets.erase(msg.timestamp);

              auto elapsed =
                  std::chrono::high_resolution_clock::now() - _startTime;
              double receiveTime =
                  std::chrono::duration_cast<std::chrono::microseconds>(elapsed)
                      .count() /
                  1000000.0;

              messageMutex->lock();
              messageQueue->push_back(msg);

              if (messageQueue == &_unconsumedMessages) {
                _messageReceiveTimes.push_back(receiveTime);
              }
              messageMutex->unlock();
              packetRoundTrip.AddData(
                  float(std::chrono::duration_cast<std::chrono::microseconds>(
                            elapsed)
                            .count() -
                        msg.timestamp) /
                  1000);
            }
            if (_logger != nullptr) {
              auto logElapsed =
                  std::chrono::high_resolution_clock::now() - _startTime;
              _logger->UpdateRxLog(
                  std::chrono::duration_cast<std::chrono::microseconds>(
                      logElapsed)
                      .count(),
                  msg);
            }
            _outstandingPacketMutex.unlock();
            radioStatsMutex->lock();
            lastReceivedTimer->markStart();
            if (msg.type == RobotMessageType::RADIO_DATA) {
              *radioStats = msg;
            }
            radioStatsMutex->unlock();
          } else {
            std::cerr << "ERROR: invalid message type" << std::endl;
          }
        } else if (recvTimeout.getElapsedTime() > 1) {
          dev->ResetRecv();
          recvTimeout.markStart();
        }
      }
    }
  }
}

void RobotLinkReal::RadioThreadFunction(void) {
  int devices_opened = 0;

  _transmitterConnected = false;
  _secondaryTransmitterConnected = false;
  _radio_reinit = false;

  Clock secondaryRetryTimer;

  std::thread primayRadioSendThread(&RobotLinkReal::RadioThreadSendFunction,
                                    this, &_radios[0], &_requestSend,
                                    &_messageToSend, &_sendMessageMutex, false);
  std::thread primaryRadioRecvThread(
      &RobotLinkReal::RadioThreadRecvFunction, this, &_radios[0],
      &_unconsumedMessagesMutex, &_unconsumedMessages, &_primaryRadioStats,
      &_primaryReceivedTimer, &_primaryRadioStatsMutex);

  std::thread secondaryRadioSendThread(
      &RobotLinkReal::RadioThreadSendFunction, this, &_radios[1],
      &_secondaryRequestSend, &_secondaryMessageToSend,
      &_secondarySendMessageMutex, true);
  std::thread secondaryRadioRecvThread(
      &RobotLinkReal::RadioThreadRecvFunction, this, &_radios[1],
      &_unconsumedMessagesMutex, &_unconsumedMessages, &_secondaryRadioStats,
      &_secondaryReceivedTimer, &_secondaryRadioStatsMutex);
  while (true) {
    while (devices_opened < 1) {
      devices_opened = RawHID_Open(_radios, 2, 0x16C0, 0x0486, 0xFFAB, 0x0200);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_RETRY_TIME));
    }
    _transmitterConnected = true;
    _secondaryTransmitterConnected = (bool)(devices_opened > 1);
    _radio_reinit = false;
    printf("Radio Reinit\n");
    if (!_secondaryTransmitterConnected) {
      secondaryRetryTimer.markStart();
    } else {
      secondaryRetryTimer.markEnd();
    }
    while (!_radio_reinit) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if (secondaryRetryTimer.isRunning() &&
          secondaryRetryTimer.getElapsedTime() > 5) {
        break;
      }
    }
    _radios[0].Close();
    _radios[1].Close();
    _transmitterConnected = false;
    _secondaryTransmitterConnected = false;
    devices_opened = 0;
  }
}

RobotLinkReal::RobotLinkReal()
    : _requestSend(false), _secondaryRequestSend(false) {
  memset(&_lastCANMessage, 0, sizeof(_lastCANMessage));
  _startTime = std::chrono::high_resolution_clock::now();
  _radioThread = std::thread(&RobotLinkReal::RadioThreadFunction, this);
}

void RobotLinkReal::RegisterLogger(DriverStationLog *logger) {
  _logger = logger;
}

/**
 * single link behaviour: rotate 1->2->3 if channel unstable
 * double link behaviour: switch worse link to unused receiver teensy
 */
void RobotLinkReal::ChooseBestChannel(DriverStationMessage &msg) {
  static bool switchedPrimary = false;
  static bool switchedSecondary = false;

  if (_lastRadioSwitchClock.getElapsedTime() * 1000 < SWITCH_COOLDOWN_MS) {
    cv::Mat &drawingImage = RobotController::GetInstance().GetDrawingImage();

    if (switchedPrimary) {
      cv::putText(drawingImage,
                  "Switched Primary to " + std::to_string(RADIO_CHANNEL),
                  cv::Point(WIDTH / 2 - 200, 50), cv::FONT_HERSHEY_SIMPLEX, 1,
                  cv::Scalar(0, 0, 255), 2);
    } else if (switchedSecondary) {
      cv::putText(
          drawingImage,
          "Switched Secondary to " + std::to_string(SECONDARY_RADIO_CHANNEL),
          cv::Point(WIDTH / 2 - 200, 50), cv::FONT_HERSHEY_SIMPLEX, 1,
          cv::Scalar(0, 0, 255), 2);
    }
  }

  if (!AUTO_SWITCH_CHANNEL) return;
  if (!_transmitterConnected) return;
  if (_lastRadioSwitchClock.getElapsedTime() * 1000 < SWITCH_COOLDOWN_MS)
    return;

  int32_t nextChannel = -1;

  switch (RADIO_CHANNEL) {
    case TEENSY_RADIO_1:
      nextChannel = TEENSY_RADIO_2;
      break;
    case TEENSY_RADIO_2:
      nextChannel = TEENSY_RADIO_3;
      break;
    case TEENSY_RADIO_3:
      nextChannel = TEENSY_RADIO_4;
      break;
    case TEENSY_RADIO_4:
      nextChannel = TEENSY_RADIO_1;
      break;
    default:
      nextChannel = TEENSY_RADIO_1;
      break;
  }
  if (_transmitterConnected && _secondaryTransmitterConnected) {
    for (uint8_t i = 0; i < 4; i++) {
      if (RADIO_CHANNEL == nextChannel ||
          SECONDARY_RADIO_CHANNEL == nextChannel) {
        switch (nextChannel) {
          case TEENSY_RADIO_1:
            nextChannel = TEENSY_RADIO_2;
            break;
          case TEENSY_RADIO_2:
            nextChannel = TEENSY_RADIO_3;
            break;
          case TEENSY_RADIO_3:
            nextChannel = TEENSY_RADIO_4;
            break;
          case TEENSY_RADIO_4:
            nextChannel = TEENSY_RADIO_1;
            break;
          default:
            nextChannel = TEENSY_RADIO_1;
            break;
        }
      } else {
        break;
      }
    }
  }

  _primaryRadioStatsMutex.lock();
  float primaryAvgDelay = _primaryRadioStats.radioData.averageDelayMS;
  _primaryRadioStatsMutex.unlock();

  _secondaryRadioStatsMutex.lock();
  float secondaryAvgDelay = _secondaryRadioStats.radioData.averageDelayMS;
  _secondaryRadioStatsMutex.unlock();

  printf("%f %f\n", _primaryReceivedTimer.getElapsedTime(),
         _secondaryReceivedTimer.getElapsedTime());

  if (_primaryReceivedTimer.getElapsedTime() * 1000 >
      PACKET_LOSS_SWITCH_TIMEOUT_MS) {
    switchedPrimary = true;
    switchedSecondary = false;
    _lastRadioSwitchClock.markStart();
    RADIO_CHANNEL = nextChannel;
  } else if (_secondaryReceivedTimer.getElapsedTime() * 1000 >
             PACKET_LOSS_SWITCH_TIMEOUT_MS) {
    switchedPrimary = false;
    switchedSecondary = true;
    _lastRadioSwitchClock.markStart();
    SECONDARY_RADIO_CHANNEL = nextChannel;
  } else if (true) {
    return;
  } else if (primaryAvgDelay > MAX_AVERAGE_DELAY_MS &&
             secondaryAvgDelay > MAX_AVERAGE_DELAY_MS) {
    _lastRadioSwitchClock.markStart();
    if (primaryAvgDelay > secondaryAvgDelay) {
      switchedPrimary = true;
      switchedSecondary = false;
      RADIO_CHANNEL = nextChannel;
    } else {
      switchedPrimary = false;
      switchedSecondary = true;
      SECONDARY_RADIO_CHANNEL = nextChannel;
    }
  } else if (primaryAvgDelay > MAX_AVERAGE_DELAY_MS) {
    switchedPrimary = true;
    switchedSecondary = false;
    _lastRadioSwitchClock.markStart();
    RADIO_CHANNEL = nextChannel;
  } else if (secondaryAvgDelay > MAX_AVERAGE_DELAY_MS) {
    switchedPrimary = false;
    switchedSecondary = true;
    _lastRadioSwitchClock.markStart();
    SECONDARY_RADIO_CHANNEL = nextChannel;
  }
}

void RobotLinkReal::Drive(DriverStationMessage &command) {
  static ClockWidget clockWidget{"Send drive command"};
  static ClockWidget interSendTime("Inter Send Time");
  static GraphWidget _radioPacketLoss("Radio Packet Loss", 0, 20, "", 100);
  static Clock packetLossUpdateRate;

  ChooseBestChannel(command);
  command.radioChannel = RADIO_CHANNEL;
  if (RESET_IMU) {
    command.resetIMU = true;
    RESET_IMU = false;
    printf("resetting imu\n");
  }

  command.fuseIMU = FUSE_IMU;

  if (_sendClock.getElapsedTime() * 1000 < MIN_INTER_SEND_TIME_MS) {
    return;
  }
  static GraphWidget stickX("Applied Movement", -1, 1, "");
  static GraphWidget stickY("Applied Turn", -1, 1, "");

  if (command.type == DRIVE_COMMAND) {
    stickX.AddData(command.driveCommand.movement /
                   (MASTER_MOVE_SCALE_PERCENT / 100.0));
    stickY.AddData(command.driveCommand.turn /
                   (MASTER_TURN_SCALE_PERCENT / 100.00));
  } else if (command.type == INVALID_DS) {
    std::cerr << "ERROR: invalid driver station message type" << std::endl;
    return;
  }

  clockWidget.markStart();

  auto elapsed = std::chrono::high_resolution_clock::now() - _startTime;
  command.timestamp =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();

  command.valid = true;

  try {
    _sendMessageMutex.lock();
    _messageToSend = command;
    _requestSend = true;
    _sendMessageMutex.unlock();

    _secondarySendMessageMutex.lock();
    _secondaryMessageToSend = command;
    _secondaryMessageToSend.radioChannel = SECONDARY_RADIO_CHANNEL;
    _secondaryRequestSend = true;
    _secondarySendMessageMutex.unlock();

    _sendClock.markStart();
    interSendTime.markEnd();
    interSendTime.markStart();

    _outstandingPacketMutex.lock();
    _outstandingPackets.insert(command.timestamp);
    _outstandingPacketMutex.unlock();
    if (_logger != nullptr) {
      _logger->UpdateTxLog(command.timestamp, command);
    }
  } catch (std::exception &e) {
    std::cerr << "Error sending message: " << e.what() << std::endl;
  }

  if (!packetLossUpdateRate.isRunning() ||
      packetLossUpdateRate.getElapsedTime() > 0.1) {
    _outstandingPacketMutex.lock();
    _radioPacketLoss.AddData(_outstandingPackets.size() -
                             _outstandingPacketCouner);
    _outstandingPacketCouner = _outstandingPackets.size();
    _outstandingPacketMutex.unlock();
    packetLossUpdateRate.markStart();
  }

  clockWidget.markEnd();
}

std::vector<RobotMessage> RobotLinkReal::_ReceiveImpl() {
  std::vector<RobotMessage> ret = {};

  _unconsumedMessagesMutex.lock();
  std::copy(_unconsumedMessages.begin(), _unconsumedMessages.end(),
            std::back_inserter(ret));
  _unconsumedMessages.clear();

  std::vector<double> receiveTimes;
  std::copy(_messageReceiveTimes.begin(), _messageReceiveTimes.end(),
            std::back_inserter(receiveTimes));
  _messageReceiveTimes.clear();

  _unconsumedMessagesMutex.unlock();

  _lastMessageReceiveTimes = receiveTimes;

  return ret;
}

RobotLinkReal::~RobotLinkReal() {}
#endif
