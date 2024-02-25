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

    if (!hasAddedUI)
    {
        deltaAngleWidget.AddAdditionalUI([]() {
            ImGui::SliderInt("Turn Thresh 1 Deg", &TURN_THRESH_1_DEG_ORBIT, 0, 360);
            ImGui::SliderInt("Turn Thresh 2 Deg", &TURN_THRESH_2_DEG_ORBIT, 0, 360);
            ImGui::SliderInt("Min Turn Power (%)", &MIN_TURN_POWER_PERCENT_ORBIT, 0, 100);
            ImGui::SliderInt("Max Turn Power (%)", &MAX_TURN_POWER_PERCENT_ORBIT, 0, 100);
            ImGui::SliderInt("Scale Down Movement (%)", &SCALE_DOWN_MOVEMENT_PERCENT_ORBIT, 0, 100);
            ImGui::SliderInt("Position Extrapolate (ms)", &POSITION_EXTRAPOLATE_MS, 0, 1000);
        });
        hasAddedUI = true;
    }


    // draw the delta angle and the thresh curve
    cv::Mat threshCurve = cv::Mat::zeros(cv::Size(400, 130), CV_8UC3);

    // increment angle from - pi to pi
    for (int i = 0; i < 400; i++)
    {
        // 0 angle in middle, pi at each end
        double angle = (i - 200) * M_PI / 200;
        double power = DoubleThreshToTarget(angle, TURN_THRESH_1_DEG_ORBIT * TO_RAD,
                                            TURN_THRESH_2_DEG_ORBIT * TO_RAD,
                                            MIN_TURN_POWER_PERCENT_ORBIT / 100.0,
                                            MAX_TURN_POWER_PERCENT_ORBIT / 100.0);
        int y = 100 - abs(power) * 100;

        y = max(0, min(199, y));
        threshCurve.at<cv::Vec3b>(y, i) = cv::Vec3b(255, 255, 255);
    }

    double power = DoubleThreshToTarget(value, TURN_THRESH_1_DEG_ORBIT * TO_RAD,
                                        TURN_THRESH_2_DEG_ORBIT * TO_RAD,
                                        MIN_TURN_POWER_PERCENT_ORBIT / 100.0,
                                        MAX_TURN_POWER_PERCENT_ORBIT / 100.0);

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

    deltaAngleWidget.UpdateMat(threshCurve);
}

/**
 * Drive the robot to a specified position
 * @param targetPos The target position
 * @param direction The direction to drive in (forward, backward, or auto)
 */
DriveCommand RobotMovement::DriveToPosition(RobotSimState exState,
                                            const cv::Point2f &targetPos,
                                            int TURN_THRESH_1_DEG,
                                            int TURN_THRESH_2_DEG,
                                            int MAX_TURN_POWER_PERCENT,
                                            int MIN_TURN_POWER_PERCENT,
                                            DriveDirection direction)
{
    static Clock c;

    cv::Point2f currPos = RobotOdometry::Robot().GetPosition();
    double currAngle = RobotOdometry::Robot().GetAngle();

    double deltaTime = c.getElapsedTime();
    c.markStart();

    double currAngleEx = exState.angle;
    cv::Point2f currPosEx = exState.position;

    double angleToTarget1 = atan2(targetPos.y - currPosEx.y, targetPos.x - currPosEx.x);
    double angleToTarget2 = angle_wrap(angleToTarget1 + M_PI);
    double deltaAngleRad1 = angle_wrap(angleToTarget1 - currAngleEx);
    double deltaAngleRad2 = angle_wrap(angleToTarget2 - currAngleEx);
    double deltaAngleRad1_noex = angle_wrap(angleToTarget1 - currAngle);
    double deltaAngleRad2_noex = angle_wrap(angleToTarget2 - currAngle);

    bool curr_direction = direction == DriveDirection::Forward ? true : false;
    // for auto, choose the direction that is closer to the target angle
    if (direction == DriveDirection::Auto)
    {
        curr_direction = abs(deltaAngleRad1_noex) < abs(deltaAngleRad2_noex);
        // put text on drawing image of curr_direction
        cv::putText(RobotController::GetInstance().GetDrawingImage(), curr_direction ? "1" : "2", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    }

    double deltaAngleRad = curr_direction ? deltaAngleRad1 : deltaAngleRad2;

    DriveCommand response{0, 0};
    response.turn = DoubleThreshToTarget(deltaAngleRad, TURN_THRESH_1_DEG * TO_RAD,
                                         TURN_THRESH_2_DEG * TO_RAD,
                                         MIN_TURN_POWER_PERCENT / 100.0,
                                         MAX_TURN_POWER_PERCENT / 100.0);

    double scaleDownMovement = SCALE_DOWN_MOVEMENT_PERCENT_ORBIT / 100.0;
    // Slow down when far away from the target angle
    double drive_scale = max(0.0, 1.0 - abs(response.turn / (MAX_TURN_POWER_PERCENT / 100.0)) * scaleDownMovement) * 1.0;

    response.movement = curr_direction ? drive_scale : -drive_scale;
    response.turn *= -1;

    DrawDeltaAngleGraphic(deltaAngleRad);

    return response;
}
