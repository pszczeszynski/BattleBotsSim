#include "IMUWidget.h"
#include "../RobotController.h"
#include "../GuiUtils.h"
#include "imgui.h"

#define REFRESH_INTERVAL_MS 30
#define WIDGET_WIDTH 250
#define WIDGET_HEIGHT 250
#define WIDGET_RADIUS WIDGET_WIDTH / 2

// the maximum acceleration in m/s^2
#define MAX_ACCEL_MPSS 15


IMUWidget::IMUWidget() : ImageWidget("IMU", false)
{

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

    // draw circle
    cv::Scalar strokeColor = cv::Scalar(255, 255, 255, 255);
    cv::circle(mat, midPoint, WIDGET_RADIUS, strokeColor, 2);

    // draw crosshair
    cv::line(mat, cv::Point(0, WIDGET_RADIUS),
             cv::Point(WIDGET_WIDTH, WIDGET_RADIUS), strokeColor);
    cv::line(mat, cv::Point(WIDGET_RADIUS, 0),
             cv::Point(WIDGET_RADIUS, WIDGET_HEIGHT), strokeColor);

    // draw blue dot
    cv::Scalar blue = cv::Scalar(0, 255, 255, 255);
    cv::Point2f dotCenter(WIDGET_RADIUS + (accelX / MAX_ACCEL_MPSS) * WIDGET_RADIUS,
                          WIDGET_RADIUS + (accelY / MAX_ACCEL_MPSS) * WIDGET_RADIUS);
    cv::circle(mat, dotCenter, 15, blue, 3);

    /////////// GYRO VISUALIZATION ///////////
    // get the gyro data
    float rotation = imuData.rotation;

    // draw a dotted circle, rotated by the rotation
    cv::Scalar dottedCircleColor = cv::Scalar(255, 255, 255, 255);
    cv::Point2f center(WIDGET_RADIUS, WIDGET_RADIUS);
    float radius = WIDGET_RADIUS - 20;
    cv::Point2f point1(center.x + radius * cos(rotation),
                       center.y + radius * sin(rotation));
    cv::Point2f point2(center.x + radius * cos(rotation + M_PI / 2),
                       center.y + radius * sin(rotation + M_PI / 2));
    cv::Point2f point3(center.x + radius * cos(rotation + M_PI),
                       center.y + radius * sin(rotation + M_PI));
    cv::Point2f point4(center.x + radius * cos(rotation + 3 * M_PI / 2),
                       center.y + radius * sin(rotation + 3 * M_PI / 2));
    cv::line(mat, point1, point3, dottedCircleColor, 1, cv::LINE_AA);
    cv::line(mat, point2, point4, dottedCircleColor, 1, cv::LINE_AA);

    // draw the widget
    ImageWidget::UpdateMat(mat);
    ImageWidget::Draw();
}