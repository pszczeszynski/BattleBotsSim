#include "OdometryPoller.h"

#include "UIWidgets/TrackingWidget.h"

namespace {
// Reference to a single data stream (us or them) for polling
struct StreamRef {
  OdometryData& out;
  const OdometryData& prev;
  uint32_t flag;
  bool isOpponent;
};

// Helper to poll a single stream from a tracker
template <typename T>
bool PollStream(T& tracker, const char* debugName, const StreamRef& stream,
                uint32_t& updatedMask, TrackingWidget* trackingInfo) {
  // Default: hold previous data
  stream.out = stream.prev;

  if (!tracker.IsRunning()) {
    return false;  // Not running, data stays as prev
  }

  // Only check for new data if flag is set
  if (stream.flag != 0 &&
      tracker.HasNewerDataById(stream.prev.id, stream.isOpponent)) {
    stream.out = tracker.GetData(stream.isOpponent);
    updatedMask |= stream.flag;

    // Fetch debug image when new data arrives
    if (trackingInfo) {
      tracker.GetDebugImage(trackingInfo->GetDebugImage(debugName),
                            trackingInfo->GetDebugOffset(debugName));
    }

    return true;
  }

  return false;
}

}  // namespace

OdometryPoller::OdometryPoller(BlobDetection& blob,
                               HeuristicOdometry& heuristic, CVPosition& neural,
                               CVRotation& neuralrot, OdometryIMU& imu,
                               HumanPosition& human,
                               HumanPosition& human_heuristic,
                               LKFlowTracker& lkflow
#ifdef USE_OPENCV_TRACKER
                               ,
                               OpenCVTracker& opencv
#endif
                               )
    : _odometry_Blob(blob),
      _odometry_Heuristic(heuristic),
      _odometry_Neural(neural),
      _odometry_NeuralRot(neuralrot),
      _odometry_IMU(imu),
      _odometry_Human(human),
      _odometry_Human_Heuristic(human_heuristic),
      _odometry_LKFlow(lkflow)
#ifdef USE_OPENCV_TRACKER
      ,
      _odometry_opencv(opencv)
#endif
{
}

RawInputs OdometryPoller::Poll(TrackingWidget* trackingInfo,
                               const RawInputs& prevInputs) {
  // Explicitly zero-initialize to prevent "ghost fusion" bugs
  RawInputs inputs{};

  // Poll Blob tracker (us and them)
  PollStream(
      _odometry_Blob, "Blob",
      StreamRef{inputs.us_blob, prevInputs.us_blob, RawInputs::US_BLOB, false},
      inputs.updatedMask, trackingInfo);
  PollStream(_odometry_Blob, "Blob",
             StreamRef{inputs.them_blob, prevInputs.them_blob,
                       RawInputs::THEM_BLOB, true},
             inputs.updatedMask, trackingInfo);

  // Poll Heuristic tracker (us and them)
  PollStream(_odometry_Heuristic, "Heuristic",
             StreamRef{inputs.us_heuristic, prevInputs.us_heuristic,
                       RawInputs::US_HEURISTIC, false},
             inputs.updatedMask, trackingInfo);
  PollStream(_odometry_Heuristic, "Heuristic",
             StreamRef{inputs.them_heuristic, prevInputs.them_heuristic,
                       RawInputs::THEM_HEURISTIC, true},
             inputs.updatedMask, trackingInfo);

#ifdef USE_OPENCV_TRACKER
  // Poll OpenCV tracker (us and them)
  PollStream(_odometry_opencv, "Opencv",
             StreamRef{inputs.us_opencv, prevInputs.us_opencv,
                       RawInputs::US_OPENCV, false},
             inputs.updatedMask, inputs.runningMask, trackingInfo);
  PollStream(_odometry_opencv, "Opencv",
             StreamRef{inputs.them_opencv, prevInputs.them_opencv,
                       RawInputs::THEM_OPENCV, true},
             inputs.updatedMask, inputs.runningMask, trackingInfo);
#endif

  // Poll IMU tracker (us only)
  PollStream(
      _odometry_IMU, "IMU",
      StreamRef{inputs.us_imu, prevInputs.us_imu, RawInputs::US_IMU, false},
      inputs.updatedMask, trackingInfo);

  // Poll NeuralRot tracker (us only)
  PollStream(_odometry_NeuralRot, "NeuralRot",
             StreamRef{inputs.us_neuralrot, prevInputs.us_neuralrot,
                       RawInputs::US_NEURALROT, false},
             inputs.updatedMask, trackingInfo);

  // Poll LKFlow tracker (them only)
  PollStream(_odometry_LKFlow, "LKFlow",
             StreamRef{inputs.them_lkflow, prevInputs.them_lkflow,
                       RawInputs::THEM_LKFLOW, true},
             inputs.updatedMask, trackingInfo);

  // Poll Neural Position (special case: always returns data when running)
  inputs.us_neural = prevInputs.us_neural;
  if (_odometry_Neural.IsRunning()) {
    inputs.us_neural = _odometry_Neural.GetData(false);
    inputs.updatedMask |= RawInputs::US_NEURAL;

    if (trackingInfo) {
      _odometry_Neural.GetDebugImage(trackingInfo->GetDebugImage("Neural"),
                                     trackingInfo->GetDebugOffset("Neural"));
    }
  }

  // Poll Human Interface (special case: tracks is_new flags)
  inputs.us_human = prevInputs.us_human;
  inputs.them_human = prevInputs.them_human;
  bool us_human_updated =
      PollStream(_odometry_Human, "Human",
                 StreamRef{inputs.us_human, prevInputs.us_human,
                           RawInputs::US_HUMAN, false},
                 inputs.updatedMask, trackingInfo);
  bool them_human_updated =
      PollStream(_odometry_Human, "Human",
                 StreamRef{inputs.them_human, prevInputs.them_human,
                           RawInputs::THEM_HUMAN, true},
                 inputs.updatedMask, trackingInfo);

  // Set is_new flags based on whether data was updated
  inputs.us_human_is_new = us_human_updated;
  inputs.them_human_is_new = them_human_updated;

  return inputs;
}
