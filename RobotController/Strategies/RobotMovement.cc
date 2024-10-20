#include "RobotMovement.h"
#include "../Clock.h"
#include "../RobotOdometry.h"
#include "../RobotConfig.h"
#include "../RobotController.h"
#include "../UIWidgets/ImageWidget.h"

void DrawDeltaAngleGraphic(double value)
{
    static ImageWidget deltaAngleWidget("Orbit Delta Angle");
    static bool hasAddedUI = false;
    static bool EDITING_ORBIT = false;

    if (!hasAddedUI)
    {
        deltaAngleWidget.AddAdditionalUI([]() {
            // check box to enable orbit vs killmode tuning
            ImGui::Checkbox(EDITING_ORBIT ? "Orbit Mode" : "Kill Mode", &EDITING_ORBIT);

            if (EDITING_ORBIT)
            {
                ImGui::SliderInt("Turn Thresh 1 Deg", &TURN_THRESH_1_DEG_ORBIT, 0, 360);
                ImGui::SliderInt("Turn Thresh 2 Deg", &TURN_THRESH_2_DEG_ORBIT, 0, 360);
                ImGui::SliderInt("Min Turn Power (%)", &MIN_TURN_POWER_PERCENT_ORBIT, 0, 100);
                ImGui::SliderInt("Max Turn Power (%)", &MAX_TURN_POWER_PERCENT_ORBIT, 0, 100);
                ImGui::SliderInt("KD * 100", &ORBIT_KD_PERCENT, 0, 100);

                ImGui::SliderInt("Scale Down Movement (%)", &SCALE_DOWN_MOVEMENT_PERCENT_ORBIT, 0, 100);
            }
            else
            {
                ImGui::SliderInt("Turn Thresh 1 Deg", &TURN_THRESH_1_DEG_KILL, 0, 360);
                ImGui::SliderInt("Turn Thresh 2 Deg", &TURN_THRESH_2_DEG_KILL, 0, 360);
                ImGui::SliderInt("Min Turn Power (%)", &MIN_TURN_POWER_PERCENT_KILL, 0, 100);
                ImGui::SliderInt("Max Turn Power (%)", &MAX_TURN_POWER_PERCENT_KILL, 0, 100);
                ImGui::SliderInt("KD * 100", &KILL_KD_PERCENT, 0, 100);

                ImGui::SliderInt("Scale Down Movement (%)", &SCALE_DOWN_MOVEMENT_PERCENT_KILL, 0, 100);
            }
        });
        hasAddedUI = true;
    }

    int& TURN_THRESH_1_DEG = EDITING_ORBIT ? TURN_THRESH_1_DEG_ORBIT : TURN_THRESH_1_DEG_KILL;
    int& TURN_THRESH_2_DEG = EDITING_ORBIT ? TURN_THRESH_2_DEG_ORBIT : TURN_THRESH_2_DEG_KILL;
    int& MIN_TURN_POWER_PERCENT = EDITING_ORBIT ? MIN_TURN_POWER_PERCENT_ORBIT : MIN_TURN_POWER_PERCENT_KILL;
    int& MAX_TURN_POWER_PERCENT = EDITING_ORBIT ? MAX_TURN_POWER_PERCENT_ORBIT : MAX_TURN_POWER_PERCENT_KILL;
    int& SCALE_DOWN_MOVEMENT_PERCENT = EDITING_ORBIT ? SCALE_DOWN_MOVEMENT_PERCENT_ORBIT : SCALE_DOWN_MOVEMENT_PERCENT_KILL;

    // draw the delta angle and the thresh curve
    cv::Mat threshCurve = cv::Mat::zeros(cv::Size(400, 130), CV_8UC3);

    // increment angle from - pi to pi
    for (int i = 0; i < 400; i++)
    {
        // 0 angle in middle, pi at each end
        double angle = (i - 200) * M_PI / 200;
        double power = DoubleThreshToTarget(angle, TURN_THRESH_1_DEG * TO_RAD,
                                            TURN_THRESH_2_DEG * TO_RAD,
                                            MIN_TURN_POWER_PERCENT / 100.0,
                                            MAX_TURN_POWER_PERCENT / 100.0);
        int y = 100 - abs(power) * 100;

        y = max(0, min(199, y));
        threshCurve.at<cv::Vec3b>(y, i) = cv::Vec3b(255, 255, 255);
    }

    double power = DoubleThreshToTarget(value, TURN_THRESH_1_DEG * TO_RAD,
                                        TURN_THRESH_2_DEG * TO_RAD,
                                        MIN_TURN_POWER_PERCENT / 100.0,
                                        MAX_TURN_POWER_PERCENT / 100.0);

    // plot the delta angle
    int x = 200 + value * 200 / M_PI;
    int y = 100 - abs(power) * 100;

    cv::circle(threshCurve, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);

    cv::arrowedLine(threshCurve, cv::Point(200, 100), cv::Point(x, 100), cv::Scalar(0, 255, 0), 2);

    // put text that this is applied turn power
    cv::putText(threshCurve, "Applied Turn Power", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

    // draw tick marks
    for (int i = 0; i < 400; i += 50)
    {
        cv::line(threshCurve, cv::Point(i, 100 - 5), cv::Point(i, 100 + 5), cv::Scalar(255, 255, 255), 1);
        // text with the angle in deg
        cv::putText(threshCurve, std::to_string((i - 200) * 180 / 200), cv::Point(i, 120), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 1);
    }

    // draw a vertical bar on the right side to show the slow down power
    int xSlow = 350;
    int ySlow = 100 - SCALE_DOWN_MOVEMENT_PERCENT;
    // interpolate between green and red
    cv::Scalar slowColor = cv::Scalar(0, 255, 0) * (100.0 - SCALE_DOWN_MOVEMENT_PERCENT) / 100.0 + cv::Scalar(0, 0, 255) * SCALE_DOWN_MOVEMENT_PERCENT / 100.0;
    cv::line(threshCurve, cv::Point(xSlow, ySlow), cv::Point(xSlow, 100), slowColor, 3);
    cv::putText(threshCurve, "Slow Down", cv::Point(xSlow + 5, ySlow + 9), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    deltaAngleWidget.UpdateMat(threshCurve);
}

/**
 * Drive the robot to a specified position
 * @param targetPos The target position
 * @param direction The direction to drive in (forward, backward, or auto)
 */
DriverStationMessage RobotMovement::HoldAngle(cv::Point2f currPos,
                                              cv::Point2f targetPos,
                                              unsigned short KD_PERCENT,
                                              int TURN_THRESH_1_DEG,
                                              int TURN_THRESH_2_DEG,
                                              int MAX_TURN_POWER_PERCENT,
                                              int MIN_TURN_POWER_PERCENT,
                                              int SCALE_DOWN_MOVEMENT_PERCENT,
                                              DriveDirection direction)
{

    double angleToTarget = atan2(targetPos.y - currPos.y, targetPos.x - currPos.x);

    // add 180 if driving backwards
    if (direction == DriveDirection::Backward)
    {
        angleToTarget = angle_wrap(angleToTarget + M_PI);
    }

    ////////// compute angular velocity of the target ////////////
    static double lastAngleToTarget = 0;
    static Clock lastCallClock;
    double deltaTargetAngle = angle_wrap(angleToTarget - lastAngleToTarget) /
                              lastCallClock.getElapsedTime();
    double angleToTargetVel = angle_wrap(angleToTarget - lastAngleToTarget) / lastCallClock.getElapsedTime();
    if (lastCallClock.getElapsedTime() > 0.2) { angleToTargetVel = 0; }
    lastAngleToTarget = angleToTarget;
    lastCallClock.markStart();
    //////////////////////////////////////////////////////////////

    // shift the angle to be in the teensy's frame of reference
    float angleToTargetForTeensy = angle_wrap(angleToTarget + RobotController::GetInstance().odometry.GetIMUOffset());

    DriverStationMessage response;
    response.type = AUTO_DRIVE;
    response.autoDrive.movement = RobotController::GetInstance().gamepad.GetRightStickY();
    response.autoDrive.targetAngle = angleToTargetForTeensy;
    response.autoDrive.KD_PERCENT = KD_PERCENT;
    response.autoDrive.TURN_THRESH_1_DEG = TURN_THRESH_1_DEG;
    response.autoDrive.TURN_THRESH_2_DEG = TURN_THRESH_2_DEG;
    response.autoDrive.MAX_TURN_POWER_PERCENT = MAX_TURN_POWER_PERCENT;
    response.autoDrive.MIN_TURN_POWER_PERCENT = MIN_TURN_POWER_PERCENT;
    response.autoDrive.SCALE_DOWN_MOVEMENT_PERCENT = SCALE_DOWN_MOVEMENT_PERCENT;
    float currAngle = RobotController::GetInstance().odometry.Robot().robotAngle;
    float angleVelocity = RobotController::GetInstance().odometry.Robot().robotAngleVelocity;
    DrawDeltaAngleGraphic(angle_wrap(angleToTarget - currAngle + angleVelocity));

    return response;
}
