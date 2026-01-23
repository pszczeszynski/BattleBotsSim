#include "OdometryPoller.h"

#include "UIWidgets/TrackingWidget.h"

namespace {
// Helper to poll a dual-tracker (tracks both us and them)
template <typename T>
bool PollDualTracker(T& tracker, const char* debugName, OdometryData& outUs,
                     OdometryData& outThem, const OdometryData& prevUs,
                     const OdometryData& prevThem, uint32_t usFlag,
                     uint32_t themFlag, uint32_t& outUpdatedMask,
                     uint32_t& outRunningMask, TrackingWidget* trackingInfo) {
  // Default: hold previous data (avoids duplicate assignments in branches)
  outUs = prevUs;
  outThem = prevThem;

  if (!tracker.IsRunning()) {
    return false;  // Not running, data stays as prev
  }

  // Mark streams as running
  outRunningMask |= (usFlag | themFlag);

  bool newdata = false;

  // Check robot (us) data - override if new
  if (tracker.NewDataValid(prevUs.id, false)) {
    outUs = tracker.GetData(false);
    outUpdatedMask |= usFlag;
    newdata = true;
  }

  // Check opponent (them) data - override if new
  if (tracker.NewDataValid(prevThem.id, true)) {
    outThem = tracker.GetData(true);
    outUpdatedMask |= themFlag;
    newdata = true;
  }

  // Get debug image if new data arrived
  if (newdata && trackingInfo) {
    tracker.GetDebugImage(trackingInfo->GetDebugImage(debugName),
                          trackingInfo->GetDebugOffset(debugName));
  }

  return newdata;
}

// Helper to poll a single-tracker (tracks only us or only them)
template <typename T>
bool PollSingleTracker(T& tracker, const char* debugName, OdometryData& outData,
                       const OdometryData& prevData, bool isOpponent,
                       uint32_t flag, uint32_t& outUpdatedMask,
                       uint32_t& outRunningMask, TrackingWidget* trackingInfo) {
  // Default: hold previous data (avoids duplicate assignment in branches)
  outData = prevData;

  if (!tracker.IsRunning()) {
    return false;  // Not running, data stays as prev
  }

  // Mark stream as running
  outRunningMask |= flag;

  bool newdata = false;

  // Check for new data - override if available
  if (tracker.NewDataValid(prevData.id, isOpponent)) {
    outData = tracker.GetData(isOpponent);
    outUpdatedMask |= flag;
    newdata = true;
  }

  // Get debug image if new data arrived
  if (newdata && trackingInfo) {
    tracker.GetDebugImage(trackingInfo->GetDebugImage(debugName),
                          trackingInfo->GetDebugOffset(debugName));
  }

  return newdata;
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

  // Poll dual-trackers (both us and them)
  PollDualTracker(_odometry_Blob, "Blob", inputs.us_blob, inputs.them_blob,
                  prevInputs.us_blob, prevInputs.them_blob, RawInputs::US_BLOB,
                  RawInputs::THEM_BLOB, inputs.updatedMask, inputs.runningMask,
                  trackingInfo);

  PollDualTracker(_odometry_Heuristic, "Heuristic", inputs.us_heuristic,
                  inputs.them_heuristic, prevInputs.us_heuristic,
                  prevInputs.them_heuristic, RawInputs::US_HEURISTIC,
                  RawInputs::THEM_HEURISTIC, inputs.updatedMask,
                  inputs.runningMask, trackingInfo);

#ifdef USE_OPENCV_TRACKER
  PollDualTracker(_odometry_opencv, "Opencv", inputs.us_opencv,
                  inputs.them_opencv, prevInputs.us_opencv,
                  prevInputs.them_opencv, RawInputs::US_OPENCV,
                  RawInputs::THEM_OPENCV, inputs.updatedMask,
                  inputs.runningMask, trackingInfo);
#endif

  // Poll single-trackers (us only)
  PollSingleTracker(_odometry_IMU, "IMU", inputs.us_imu, prevInputs.us_imu,
                    false, RawInputs::US_IMU, inputs.updatedMask,
                    inputs.runningMask, trackingInfo);

  PollSingleTracker(_odometry_NeuralRot, "NeuralRot", inputs.us_neuralrot,
                    prevInputs.us_neuralrot, false, RawInputs::US_NEURALROT,
                    inputs.updatedMask, inputs.runningMask, trackingInfo);

  // Poll single-tracker (them only)
  PollSingleTracker(_odometry_LKFlow, "LKFlow", inputs.them_lkflow,
                    prevInputs.them_lkflow, true, RawInputs::THEM_LKFLOW,
                    inputs.updatedMask, inputs.runningMask, trackingInfo);

  // Poll Neural Position (special case: always returns data when running)
  if (_odometry_Neural.IsRunning()) {
    inputs.runningMask |= RawInputs::US_NEURAL;
    inputs.us_neural = _odometry_Neural.GetData(false);
    inputs.updatedMask |= RawInputs::US_NEURAL;

    if (trackingInfo) {
      _odometry_Neural.GetDebugImage(trackingInfo->GetDebugImage("Neural"),
                                     trackingInfo->GetDebugOffset("Neural"));
    }
  } else {
    inputs.us_neural = prevInputs.us_neural;
  }

  // Poll Human Interface (special case: tracks is_new flags)
  if (_odometry_Human.IsRunning()) {
    inputs.runningMask |= (RawInputs::US_HUMAN | RawInputs::THEM_HUMAN);

    if (_odometry_Human.NewDataValid(prevInputs.us_human.id, false)) {
      inputs.us_human = _odometry_Human.GetData(false);
      inputs.us_human_is_new = true;
      inputs.updatedMask |= RawInputs::US_HUMAN;
    } else {
      inputs.us_human = prevInputs.us_human;
      inputs.us_human_is_new = false;
    }

    if (_odometry_Human.NewDataValid(prevInputs.them_human.id, true)) {
      inputs.them_human = _odometry_Human.GetData(true);
      inputs.them_human_is_new = true;
      inputs.updatedMask |= RawInputs::THEM_HUMAN;
    } else {
      inputs.them_human = prevInputs.them_human;
      inputs.them_human_is_new = false;
    }
  } else {
    inputs.us_human = prevInputs.us_human;
    inputs.them_human = prevInputs.them_human;
    inputs.us_human_is_new = false;
    inputs.them_human_is_new = false;
  }

  return inputs;
}
