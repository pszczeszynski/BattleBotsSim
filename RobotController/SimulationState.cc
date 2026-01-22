#include "SimulationState.h"

#include "Clock.h"


// Singleton instance getter
SimulationState& SimulationState::GetInstance() {
  static SimulationState instance;
  return instance;
}

// Thread-safe setters
void SimulationState::SetRobotData(const cv::Point2f& position,
                                   const cv::Point2f& velocity,
                                   double timestamp) {
  std::lock_guard<std::mutex> lock(_mutex);
  _robotData.position = position;
  _robotData.velocity = velocity;
  _robotData.timestamp = timestamp;
}

void SimulationState::SetOpponentData(const cv::Point2f& position,
                                      const cv::Point2f& velocity,
                                      double rotation, double rotationVelocity,
                                      double timestamp) {
  std::lock_guard<std::mutex> lock(_mutex);
  _opponentData.position = position;
  _opponentData.velocity = velocity;
  _opponentData.rotation = rotation;
  _opponentData.rotationVelocity = rotationVelocity;
  _opponentData.timestamp = timestamp;
}

// Thread-safe getters that return copies
SimulationState::RobotData SimulationState::GetRobotData() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _robotData;
}

SimulationState::RobotData SimulationState::GetOpponentData() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _opponentData;
}

