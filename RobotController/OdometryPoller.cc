#include "OdometryPoller.h"

namespace {
// Reference to a single data stream (us or them) for polling
struct StreamRef {
  OdometryData& latest;
  const OdometryData& prev;
  uint32_t flag;
  bool isOpponent;
};

bool PollStream(OdometryBase& odometry, const StreamRef& stream,
                uint32_t& updatedMask) {
  // Default: hold previous data
  stream.latest = stream.prev;

  if (!odometry.IsRunning()) {
    return false;  // Not running, data stays as prev
  }

  // Only check for new data if flag is set
  if (odometry.HasNewerDataById(stream.prev.id, stream.isOpponent)) {
    stream.latest = odometry.GetData(stream.isOpponent);
    updatedMask |= stream.flag;
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

RawInputs OdometryPoller::Poll(const RawInputs& prevInputs) {
  // Explicitly zero-initialize
  RawInputs inputs{};

  // Poll Blob tracker (us and them)
  PollStream(
      _odometry_Blob,
      StreamRef{inputs.us_blob, prevInputs.us_blob, RawInputs::US_BLOB, false},
      inputs.updatedMask);
  PollStream(_odometry_Blob,
             StreamRef{inputs.them_blob, prevInputs.them_blob,
                       RawInputs::THEM_BLOB, true},
             inputs.updatedMask);

  // Poll Heuristic tracker (us and them)
  PollStream(_odometry_Heuristic,
             StreamRef{inputs.us_heuristic, prevInputs.us_heuristic,
                       RawInputs::US_HEURISTIC, false},
             inputs.updatedMask);
  PollStream(_odometry_Heuristic,
             StreamRef{inputs.them_heuristic, prevInputs.them_heuristic,
                       RawInputs::THEM_HEURISTIC, true},
             inputs.updatedMask);

#ifdef USE_OPENCV_TRACKER
  // Poll OpenCV tracker (us and them)
  PollStream(_odometry_opencv,
             StreamRef{inputs.us_opencv, prevInputs.us_opencv,
                       RawInputs::US_OPENCV, false},
             inputs.updatedMask);
  PollStream(_odometry_opencv,
             StreamRef{inputs.them_opencv, prevInputs.them_opencv,
                       RawInputs::THEM_OPENCV, true},
             inputs.updatedMask);
#endif
  // Poll LKFlow tracker (us and them)
  PollStream(_odometry_LKFlow,
             StreamRef{inputs.us_lkflow, prevInputs.us_lkflow,
                       RawInputs::US_LKFLOW, false},
             inputs.updatedMask);
  PollStream(_odometry_LKFlow,
             StreamRef{inputs.them_lkflow, prevInputs.them_lkflow,
                       RawInputs::THEM_LKFLOW, true},
             inputs.updatedMask);

  // Poll IMU tracker (us only)
  PollStream(
      _odometry_IMU,
      StreamRef{inputs.us_imu, prevInputs.us_imu, RawInputs::US_IMU, false},
      inputs.updatedMask);

  // Poll NeuralRot tracker (us only)
  PollStream(_odometry_NeuralRot,
             StreamRef{inputs.us_neuralrot, prevInputs.us_neuralrot,
                       RawInputs::US_NEURALROT, false},
             inputs.updatedMask);

  // Poll Neural Position (special case: always returns data when running)
  inputs.us_neural = prevInputs.us_neural;
  if (_odometry_Neural.IsRunning()) {
    inputs.us_neural = _odometry_Neural.GetData(false);
    inputs.updatedMask |= RawInputs::US_NEURAL;
  }

  // Poll Human Interface (special case: tracks is_new flags)
  inputs.us_human = prevInputs.us_human;
  inputs.them_human = prevInputs.them_human;
  bool us_human_updated =
      PollStream(_odometry_Human,
                 StreamRef{inputs.us_human, prevInputs.us_human,
                           RawInputs::US_HUMAN, false},
                 inputs.updatedMask);
  bool them_human_updated =
      PollStream(_odometry_Human,
                 StreamRef{inputs.them_human, prevInputs.them_human,
                           RawInputs::THEM_HUMAN, true},
                 inputs.updatedMask);

  return inputs;
}
