#include "IMUWidget.h"
#include "../RobotController.h"
#include "../GuiUtils.h"
#include "imgui.h"
#include "../SafeDrawing.h"

#define REFRESH_INTERVAL_MS 30
#define WIDGET_WIDTH 250
#define WIDGET_HEIGHT 250
#define WIDGET_RADIUS WIDGET_WIDTH / 2

// the maximum acceleration in m/s^2
#define MAX_ACCEL_MPSS 15


IMUWidget::IMUWidget() : ImageWidget("IMU", false)
{
    AddAdditionalUI([](){
        // button
        if (ImGui::Button("Flip angle"))
        {
            RobotOdometry& odometry = RobotController::GetInstance().odometry;
            odometry.UpdateForceSetAngle(odometry.Robot().GetAngle() + M_PI, false);
        }
    });
}

void DrawCrossRotated(cv::Mat& mat, cv::Point2f center, float rotation, cv::Scalar color, float radius, int thickness = 1)
{
    cv::Point2f point1(center.x + radius * cos(rotation),
                       center.y + radius * sin(rotation));
    cv::Point2f point2(center.x + radius * cos(rotation + M_PI / 2),
                       center.y + radius * sin(rotation + M_PI / 2));
    cv::Point2f point3(center.x + radius * cos(rotation + M_PI),
                       center.y + radius * sin(rotation + M_PI));
    cv::Point2f point4(center.x + radius * cos(rotation + 3 * M_PI / 2),
                       center.y + radius * sin(rotation + 3 * M_PI / 2));
    cv::line(mat, point1, point3, color, thickness, cv::LINE_AA);
    cv::line(mat, point2, point4, color, thickness, cv::LINE_AA);
    
    // Draw arrow tip pointing in the direction the robot is facing
    float arrowLength = 20.0f;
    float arrowWidth = 8.0f;
    
    // Arrow tip point (in the direction of rotation)
    cv::Point2f arrowTip(center.x + (radius + arrowLength) * cos(rotation),
                         center.y + (radius + arrowLength) * sin(rotation));
    
    // Arrow base points (perpendicular to the direction)
    cv::Point2f arrowBase1(center.x + radius * cos(rotation + M_PI / 2),
                           center.y + radius * sin(rotation + M_PI / 2));
    cv::Point2f arrowBase2(center.x + radius * cos(rotation - M_PI / 2),
                           center.y + radius * sin(rotation - M_PI / 2));
    
    // Draw arrow lines
    cv::line(mat, arrowBase1, arrowTip, color, thickness, cv::LINE_AA);
    cv::line(mat, arrowBase2, arrowTip, color, thickness, cv::LINE_AA);
}

/**
 * \brief
 * Interpolates between red and green based on the value
 * \param value The value to interpolate between 0 and 1
*/
cv::Scalar InterpolateRedToGreen(float value)
{
    float red = 255 * (1 - value);
    float green = 255 * value;
    return cv::Scalar(0, green, red, 255);
}

void IMUWidget::Draw()
{
    // get the latest message
    IMUData imuData = RobotController::GetInstance().GetIMUData();

    // get the acceleration
    float accelX = imuData.accelX;
    float accelY = imuData.accelY;

    // get the midpoint
    cv::Point2f midPoint = cv::Point2f(WIDGET_RADIUS, WIDGET_RADIUS);

    // initialize the mat to grey
    cv::Scalar backgroundGrey = cv::Scalar(0, 0, 0, 0);
    cv::Mat mat = cv::Mat(WIDGET_HEIGHT, WIDGET_WIDTH, CV_8UC4, backgroundGrey);

    // put text
    cv::putText(mat, "IMU", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255, 255), 2);
    
    // Get IMU debug data
    RobotMessage imuDebugMessage = RobotController::GetInstance().GetRobotLink().GetLastIMUDebugMessage();
    
    // Show health status
    std::string healthText = "UNHEALTHY";
    cv::Scalar healthColor = cv::Scalar(255, 0, 0, 255); // Red by default
    
    if (imuDebugMessage.type == RobotMessageType::IMU_DEBUG_DATA)
    {
        healthText = imuDebugMessage.imuDebugData.isHealthy ? "HEALTHY" : "UNHEALTHY";
        healthColor = imuDebugMessage.imuDebugData.isHealthy ? cv::Scalar(0, 255, 0, 255) : cv::Scalar(255, 0, 0, 255);
    }
    
    cv::putText(mat, healthText, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, healthColor, 2);
    
    // Show IMU debug data as text
    if (imuDebugMessage.type == RobotMessageType::IMU_DEBUG_DATA)
    {
        int yOffset = 90;
        int lineHeight = 20;
        
        // Current rotation
        std::string rotText = "Rot: " + std::to_string(imuDebugMessage.imuDebugData.myCurrRotationRad * 180.0 / M_PI) + " deg";
        cv::putText(mat, rotText, cv::Point(10, yOffset), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255, 255), 1);
        yOffset += lineHeight;
        
        // Average velocity difference
        std::string velDiffText = "Vel Diff: " + std::to_string(imuDebugMessage.imuDebugData.avgVelocityDifference) + " rad/s";
        cv::putText(mat, velDiffText, cv::Point(10, yOffset), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255, 255), 1);
        yOffset += lineHeight;
        
        // External mean velocity
        std::string extVelText = "Ext Vel: " + std::to_string(imuDebugMessage.imuDebugData.externalMeanVelocity) + " rad/s";
        cv::putText(mat, extVelText, cv::Point(10, yOffset), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255, 255), 1);
        yOffset += lineHeight;
        
        // Percentage error
        std::string errorText = "Error: " + std::to_string(imuDebugMessage.imuDebugData.percentageError) + "%";
        cv::putText(mat, errorText, cv::Point(10, yOffset), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255, 255), 1);
    }

    // draw circle
    cv::Scalar strokeColor = cv::Scalar(255, 255, 255, 255);
    safe_circle(mat, midPoint, WIDGET_RADIUS, strokeColor, 2);

    // draw crosshair
    cv::line(mat, cv::Point(0, WIDGET_RADIUS),
             cv::Point(WIDGET_WIDTH, WIDGET_RADIUS), strokeColor);
    cv::line(mat, cv::Point(WIDGET_RADIUS, 0),
             cv::Point(WIDGET_RADIUS, WIDGET_HEIGHT), strokeColor);

    // draw blue dot
    cv::Scalar blue = cv::Scalar(0, 255, 255, 255);
    cv::Point2f dotCenter(WIDGET_RADIUS + (accelX / MAX_ACCEL_MPSS) * WIDGET_RADIUS,
                          WIDGET_RADIUS + (accelY / MAX_ACCEL_MPSS) * WIDGET_RADIUS);
    safe_circle(mat, dotCenter, 15, blue, 3);

    /////////// GYRO VISUALIZATION ///////////
    // get the gyro data
    float rotation = imuData.rotation;
    // draw a dotted cross, rotated by the rotation
    cv::Scalar crossColor = cv::Scalar(255, 255, 255, 255);
    cv::Point2f center(WIDGET_RADIUS, WIDGET_RADIUS);
    float radius = WIDGET_RADIUS - 20;

    // draw the cross
    DrawCrossRotated(mat, center, rotation, crossColor, radius);

    CVRotation* pcvRotation = CVRotation::GetInstance();
    ////////// CV ROTATION VISUALIZATION //////////
    if (pcvRotation != nullptr && pcvRotation->IsRunning())
    {
        // get the cv rotation
        double cvRotation = pcvRotation->GetLastComputedRotation();
        // draw a dotted cross, rotated by the rotation
        cv::Scalar cvCrossColor = InterpolateRedToGreen(pcvRotation->GetLastConfidence());
        cv::Point2f cvCenter(WIDGET_RADIUS, WIDGET_RADIUS);
        float cvRadius = WIDGET_RADIUS - 40;
        cvRadius *= pcvRotation->GetLastConfidence();

        // draw the dotted cross
        DrawCrossRotated(mat, cvCenter, cvRotation, cvCrossColor, cvRadius, 3);
    }
    else if (pcvRotation != nullptr)
    {
        // std::cout << "CV Rotation not running" << std::endl;
    }
    else
    {
        // std::cout << "CV Rotation is null" << std::endl;
    }

    // draw the widget
    ImageWidget::UpdateMat(mat);
    ImageWidget::Draw();
}