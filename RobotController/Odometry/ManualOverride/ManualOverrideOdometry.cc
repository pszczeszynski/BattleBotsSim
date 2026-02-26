#include "ManualOverrideOdometry.h"

ManualOverrideOdometry::ManualOverrideOdometry() : OdometryBase() {}

bool ManualOverrideOdometry::Run() {
  if (_running.load()) return false;
  _running.store(true);
  return true;
}

bool ManualOverrideOdometry::Stop() {
  _running.store(false);
  return true;
}

void ManualOverrideOdometry::SetPosition(const PositionData& newPos,
                                         bool opponentRobot) {
  std::lock_guard<std::mutex> lock(_updateMutex);
  int slot = opponentRobot ? (int)RobotSlot::Opponent : (int)RobotSlot::Us;
  PositionData posWithAlg = newPos;
  posWithAlg.algorithm = OdometryAlg::ManualOverride;
  _data[slot].pos = posWithAlg;
  _data[slot].id++;
}

void ManualOverrideOdometry::SetAngle(AngleData angleData, bool opponentRobot) {
  std::lock_guard<std::mutex> lock(_updateMutex);
  int slot = opponentRobot ? (int)RobotSlot::Opponent : (int)RobotSlot::Us;
  angleData.algorithm = OdometryAlg::ManualOverride;
  _data[slot].angle = angleData;
  _data[slot].id++;
}
