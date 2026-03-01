#pragma once

#include <cstdint>

#include "CVRotation.h"
#include "Odometry/BlobDetection/BlobDetection.h"
#include "Odometry/Heuristic1/HeuristicOdometry.h"
#include "Odometry/Human/HumanPosition.h"
#include "Odometry/IMU/OdometryIMU.h"
#include "Odometry/LKFlowTracker/LKFlowTracker.h"
#include "Odometry/ManualOverride/ManualOverrideOdometry.h"
#include "Odometry/Neural/CVPosition.h"
#include "Odometry/OpenCVTracker/OpenCVTracker.h"

/**
 * @brief Structure holding raw odometry data from all algorithms for one update
 * cycle
 *
 * Each field contains the latest data from its respective algorithm.
 * The updatedMask field uses bitflags to indicate which algorithms produced new
 * data.
 */
struct RawInputs {
  // Robot (us) data
  OdometryData us_blob;
  OdometryData us_heuristic;
  OdometryData us_neural;
  OdometryData us_neuralrot;
  OdometryData us_imu;
  OdometryData us_human;
  OdometryData us_lkflow;
  OdometryData us_override;

  // Opponent (them) data
  OdometryData them_blob;
  OdometryData them_heuristic;
  OdometryData them_human;
  OdometryData them_lkflow;
  OdometryData them_override;
  OdometryData us_opencv;
  OdometryData them_opencv;

  // Bitflags indicating which algorithms updated this cycle
  enum UpdateMask : uint32_t {
    NONE = 0,
    US_BLOB = 1 << 0,
    THEM_BLOB = 1 << 1,
    US_HEURISTIC = 1 << 2,
    THEM_HEURISTIC = 1 << 3,
    US_NEURAL = 1 << 4,
    US_NEURALROT = 1 << 5,
    US_IMU = 1 << 6,
    US_HUMAN = 1 << 7,
    THEM_HUMAN = 1 << 8,
    US_LKFLOW = 1 << 9,
    THEM_LKFLOW = 1 << 10,
    US_OVERRIDE = 1 << 11,
    THEM_OVERRIDE = 1 << 12,
    US_OPENCV = 1 << 13,
    THEM_OPENCV = 1 << 14,
  };

  uint32_t updatedMask = 0;  // In-class init to prevent uninitialized bugs

  RawInputs() = default;  // Now uses in-class initializers

  bool HasUpdates() const { return updatedMask != NONE; }
};

/**
 * @brief Polls all odometry algorithms and collects their latest data
 *
 * Responsible for:
 * - Checking if each algorithm is running
 * - Detecting if new data is available
 * - Retrieving the latest data from each algorithm
 * - Collecting debug images for the UI
 */
class OdometryPoller {
 public:
  OdometryPoller(BlobDetection& blob, HeuristicOdometry& heuristic,
                 CVPosition& neural, CVRotation& neuralrot, OdometryIMU& imu,
                 HumanPosition& human, HumanPosition& human_heuristic,
                 LKFlowTracker& lkflow, ManualOverrideOdometry& manual_override,
                 OpenCVTracker& opencv);

  /**
   * @brief Poll all odometry algorithms for new data
   *
   * @param prevInputs Previous cycle's inputs (used for tracking human
   * interface updates)
   * @return RawInputs containing all available odometry data and update flags
   */
  RawInputs Poll(const RawInputs& prevInputs);

 private:
  // References to all odometry algorithm instances
  BlobDetection& _odometry_Blob;
  HeuristicOdometry& _odometry_Heuristic;
  CVPosition& _odometry_Neural;
  CVRotation& _odometry_NeuralRot;
  OdometryIMU& _odometry_IMU;
  HumanPosition& _odometry_Human;
  HumanPosition& _odometry_Human_Heuristic;
  LKFlowTracker& _odometry_LKFlow;
  ManualOverrideOdometry& _odometry_Override;
  OpenCVTracker& _odometry_opencv;
};
