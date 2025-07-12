#pragma once
#include "CircularDeque.h"
#include "Communication.h"
#include <functional>

const std::string MESSAGE_START_SEQ = "<<<";
const std::string MESSAGE_END_SEQ = ">>>";

template <typename T> class GenericReceiver {
  enum class State { Idle, Receiving, MessageComplete, Error };

  State currentState = State::Idle;
  CircularDeque<char> serialBuffer;

  T latestData;
  bool latestDataValid = false;

public:
  GenericReceiver(int buffer_size,
                  std::function<bool(char &)> single_char_read_lambda)
      : serialBuffer(buffer_size), readChar(single_char_read_lambda) {}

private:
  std::function<bool(char &)> readChar;

  void _handleError() {
    serialBuffer.clear();
    currentState = State::Idle; // Reset state
  }

  void _processChar(char c) {
    // add char to buffer
    serialBuffer.push_back(c);

    switch (currentState) {
    case State::Idle:
      if (serialBuffer.Size() >= MESSAGE_START_SEQ.length()) {
        // Check if the last characters in the buffer match the start sequence
        if (serialBuffer.endsWith(MESSAGE_START_SEQ)) {
          currentState = State::Receiving;
        } else {
          serialBuffer.pop_front(); // Keep the buffer size equal to the start
                                    // sequence length - 1
        }
      }
      break;

    case State::Receiving:
      if (serialBuffer.endsWith(MESSAGE_END_SEQ)) {
        if (serialBuffer.Size() >=
            sizeof(T) + MESSAGE_START_SEQ.length() + MESSAGE_END_SEQ.length()) {
          processMessage();
          currentState = State::Idle; // Reset state
        } else {
          _handleError();
        }
      }
      // Additional checks can be added here for maximum message size, etc.
      break;
    }
  }

  void processMessage() {
    // Assuming message format: [START_SEQ][DATA][END_SEQ]
    serialBuffer.pop_front(MESSAGE_START_SEQ.length()); // Remove start sequence
    serialBuffer.pop_back(MESSAGE_END_SEQ.length());    // Remove end sequence
    int copy_result = serialBuffer.copy_to((char *)&latestData, 0, sizeof(T));

    if (copy_result != sizeof(T)) {
      return;
    }

    latestDataValid = true;
    serialBuffer.clear(); // Clear buffer after processing message
  }

public:
  T getLatestData() {
    T &ret = latestData;
    latestDataValid = false;
    return ret;
  }

  bool isLatestDataValid() const { return latestDataValid; }

  /**
   * @brief Reads data from the serial port until a message is received
   * @note This function blocks until a message is received
   */
  void waitUntilData() {
    char c;

    // read until we get a message
    while (!latestDataValid) {
      // keep calling read char until we get a character
      while (!readChar(c)) {
      }

      _processChar(c);
    }
  }

  /**
   * @brief Reads all available data from the serial port and processes it
   * @note This function does not block
   */
  void readUntilEnd() {
    char c;
    while (readChar(c)) {
      _processChar(c);
    }
  }
};
