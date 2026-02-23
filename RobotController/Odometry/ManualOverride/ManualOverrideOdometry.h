#pragma once

#include "../OdometryBase.h"

/**
 * Odometry source for human manual overrides (e.g. click-to-set position in
 * TrackingWidget). No camera or thread; data is set via SetPosition/SetAngle.
 * Takes priority in fusion when fresh.
 */
class ManualOverrideOdometry : public OdometryBase {
 public:
  ManualOverrideOdometry();

  bool Run() override;
  bool Stop() override;
  void SetPosition(const PositionData& newPos, bool opponentRobot) override;
  void SetAngle(AngleData angleData, bool opponentRobot) override;
};
