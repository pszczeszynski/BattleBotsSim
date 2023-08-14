#include "IMUWidget.h"
#include "../RobotController.h"
#include <QImage>
#include <QPixmap>

#define REFRESH_INTERVAL_MS 30
#define WIDGET_WIDTH 500
#define WIDGET_HEIGHT 500
#define WIDGET_RADIUS WIDGET_WIDTH / 2

// the maximum acceleration in m/s^2
#define MAX_ACCEL_MPSS 15

IMUWidget* IMUWidget::_instance = nullptr;

IMUWidget::IMUWidget(QWidget *parent) : QLabel(parent)
{
    connect(&_drawTimer, &QTimer::timeout, this, &IMUWidget::Draw);
    _drawTimer.start(REFRESH_INTERVAL_MS);

    IMUWidget::_instance = this;
}

/**
 * Returns the singleton instance of the IMUWidget
*/
IMUWidget& IMUWidget::GetInstance()
{
    return *_instance;
}

void IMUWidget::SetMat(const cv::Mat& mat)
{
    _matMutex.lock();
    _mat = mat.clone();
    _matMutex.unlock();
}

/**
 * Draws the new drawing image
 */
void IMUWidget::Update()
{

    // get the latest message
    RobotMessage msg = RobotController::GetInstance().GetLatestMessage();
    // get the acceleration
    float accelX = msg.accelX;
    float accelY = msg.accelY;

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

    // draw velocity
    float velocityX = msg.velocityX;
    float velocityY = msg.velocityY;
    cv::Point2f velocity(WIDGET_RADIUS + (velocityX / 3) * WIDGET_RADIUS,
                         WIDGET_RADIUS + (velocityY / 3) * WIDGET_RADIUS);
    cv::line(mat, midPoint, velocity, blue, 3);





    /////////// GYRO VISUALIZATION ///////////
    // get the gyro data
    float rotation = msg.rotation;

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



    // lock the mutex
    _matMutex.lock();
    // copy the mat
    mat.copyTo(_mat);
    // unlock the mutex
    _matMutex.unlock();
}

void IMUWidget::Draw()
{
    // if the mat is empty, do nothing
    if (_mat.empty())
    {
        return;
    }

    // lock the mutex
    _matMutex.lock();
    // Convert to QImage and set it as the image in QLabel
    QImage imageQt(_mat.data, _mat.cols, _mat.rows, QImage::Format_RGBA8888);
    QPixmap pixmap = QPixmap::fromImage(imageQt);
    this->setPixmap(pixmap.scaled(this->size(), Qt::KeepAspectRatio));
    // unlock the mutex
    _matMutex.unlock();
}
