#include "RobotMovement.h"
#include "../Common/Communication.h"
#include "Clock.h"
#include "MathUtils.h"
#include "UIWidgets/GraphWidget.h"
#include "RobotConfig.h"

/**
 * Drives the robot to a target angle given
 * 1. The current angle
 * 2. Our angular velocity (from the gyro)
 * 3. The target angle
 * 4. The target angular velocity, used to compute the D term
 * 5. The KD term as a percentage (may be over 100)
 * 6. The first threshold for turning in degrees
 * 7. The second threshold for turning in degrees
 * 8. The max power to turn at (100 is full power)
 * 9. The min power to turn at (100 is full power)
 * 10. The scale down movement percent (100 is full power)
 */
DriveCommand DriveToAngle(double currAngle,
                          float targetAngle,
                          unsigned short KD_PERCENT,
                          short TURN_THRESH_1_DEG,
                          short TURN_THRESH_2_DEG,
                          short MAX_TURN_POWER_PERCENT,
                          short MIN_TURN_POWER_PERCENT,
                          short SCALE_DOWN_MOVEMENT_PERCENT,
                          float movement) {

  static GraphWidget stickX("Applied Movement", -1, 1, "");
  static GraphWidget stickY("Applied Turn", -1, 1, "");
  static GraphWidget derivative("Derivative", -1, 1, "");

  static double lastTime = Clock::programClock.getElapsedTime();
  static double lastError = 0.0;
  static double lastDerivative = 0.0;

  double currTime = Clock::programClock.getElapsedTime();
  double deltaTimeSeconds = currTime - lastTime;
  lastTime = currTime;

  // Calculate the angle to the target
  double deltaAngleRad = angle_wrap(targetAngle - currAngle);

  // Calculate the power to turn at
  DriveCommand ret{0, 0, 0, 0, 0};
  ret.turn = DoubleThreshToTarget(
      deltaAngleRad, TURN_THRESH_1_DEG * TO_RAD, TURN_THRESH_2_DEG * TO_RAD,
      MIN_TURN_POWER_PERCENT / 100.0, MAX_TURN_POWER_PERCENT / 100.0);

  // Calculate derivative term from error change over time
  double errorChange = deltaAngleRad - lastError;
  double deltaAngleVel = deltaTimeSeconds > 0 ? errorChange / deltaTimeSeconds : 0.0;


  double currDerivative = deltaAngleVel;
  if (KD_FILTER_TIME_CONSTANT > 0) {
    currDerivative = Interpolate(lastDerivative, deltaAngleVel, deltaTimeSeconds / KD_FILTER_TIME_CONSTANT);
  }
  lastDerivative = currDerivative;

  lastError = deltaAngleRad;
  ret.turn += currDerivative * (KD_PERCENT / 100.0) * 0.3; // add the D term
  ret.movement = movement;
  // clip turn
  ret.turn = std::clamp(ret.turn, -1.0f, 1.0f);

  float total = abs(ret.movement) + abs(ret.turn);
  if (total > 1) {
    float remaining_for_movement = 1.0f - std::abs(ret.turn);
    float fully_clamped_movement = std::clamp(ret.movement, -remaining_for_movement, remaining_for_movement);
    float fully_clamped_turn = ret.turn;
    float fixed_scale_movement = ret.movement * (1.0 / total);
    float fixed_scale_turn = ret.turn * (1.0 / total);

    float scale_down_movement_frac = SCALE_DOWN_MOVEMENT_PERCENT / 100.0f;
    float interpolated_movement = Interpolate(fixed_scale_movement, fully_clamped_movement, scale_down_movement_frac);
    float interpolated_turn = Interpolate(fixed_scale_turn, fully_clamped_turn, scale_down_movement_frac);

    ret.movement = interpolated_movement;
    ret.turn = interpolated_turn;
  }

  stickX.AddData(ret.movement);
  stickY.AddData(ret.turn);
  derivative.AddData(currDerivative * (KD_PERCENT / 100.0) * 0.3);

  return ret;
}
