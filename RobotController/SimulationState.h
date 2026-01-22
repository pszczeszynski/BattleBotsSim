#pragma once

#include <mutex>
#include <opencv2/opencv.hpp>


/**
 * @brief Thread-safe singleton for managing simulation state data
 */
class SimulationState {
 public:
  struct RobotData {
    cv::Point2f position{0, 0};
    cv::Point2f velocity{0, 0};
    double rotation{0.0};
    double rotationVelocity{0.0};
    double timestamp{0.0};
  };

  // Get singleton instance
  static SimulationState& GetInstance();

  // Delete copy/move constructors and operators
  SimulationState(const SimulationState&) = delete;
  SimulationState& operator=(const SimulationState&) = delete;
  SimulationState(SimulationState&&) = delete;
  SimulationState& operator=(SimulationState&&) = delete;

  // Thread-safe setters
  void SetRobotData(const cv::Point2f& position, const cv::Point2f& velocity,
                    double timestamp);
  void SetOpponentData(const cv::Point2f& position, const cv::Point2f& velocity,
                       double rotation, double rotationVelocity,
                       double timestamp);

  // Thread-safe getters that return copies
  RobotData GetRobotData() const;
  RobotData GetOpponentData() const;

 private:
  SimulationState() = default;
  ~SimulationState() = default;

  mutable std::mutex _mutex;
  RobotData _robotData;
  RobotData _opponentData;
};
