#pragma once

#include <cstdint>

#include "CVRotation.h"
#include "Odometry/BlobDetection/BlobDetection.h"
#include "Odometry/Heuristic1/HeuristicOdometry.h"
#include "Odometry/Human/HumanPosition.h"
#include "Odometry/IMU/OdometryIMU.h"
#include "Odometry/LKFlowTracker/LKFlowTracker.h"
#include "Odometry/Neural/CVPosition.h"
#include "Odometry/OdometryBase.h"


#ifdef USE_OPENCV_TRACKER
#include "Odometry/OpenCVTracker/OpenCVTracker.h"
#endif

class TrackingWidget;

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

  // Opponent (them) data
  OdometryData them_blob;
  OdometryData them_heuristic;
  OdometryData them_human;
  OdometryData them_lkflow;

#ifdef USE_OPENCV_TRACKER
  OdometryData us_opencv;
  OdometryData them_opencv;
#endif

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
    THEM_LKFLOW = 1 << 9,
#ifdef USE_OPENCV_TRACKER
    US_OPENCV = 1 << 10,
    THEM_OPENCV = 1 << 11,
#endif
  };

  uint32_t updatedMask = 0;  // In-class init to prevent uninitialized bugs

  // Flags for human interface
  bool us_human_is_new = false;
  bool them_human_is_new = false;

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
                 LKFlowTracker& lkflow
#ifdef USE_OPENCV_TRACKER
                 ,
                 OpenCVTracker& opencv
#endif
  );

  /**
   * @brief Poll all odometry algorithms for new data
   *
   * @param trackingInfo UI widget for debug visualization (can be nullptr)
   * @param prevInputs Previous cycle's inputs (used for tracking human
   * interface updates)
   * @return RawInputs containing all available odometry data and update flags
   */
  RawInputs Poll(TrackingWidget* trackingInfo, const RawInputs& prevInputs);

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

#ifdef USE_OPENCV_TRACKER
  OpenCVTracker& _odometry_opencv;
#endif
};
