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

#define AMPS_WARN 100
#define AMPS_RED 200
#define VOLT_WARN 60
#define VOLT_RED 58
#define TEMP_WARN 70
#define TEMP_RED 80

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

void IMUWidget::_UpdateVESCInfos()
{
    CANData canData = RobotController::GetInstance().GetCANData();

    for (int i = 0; i < 4; i++)
    {
        QLabel *label = RobotControllerGUI::GetInstance().GetVescInfo(i, 0);
        QPalette palette = label->palette();

        if (canData.motorCurrent[i] < AMPS_WARN)
            palette.setColor(QPalette::WindowText, QColor(0, 255, 0));
        else if (canData.motorCurrent[i] < AMPS_RED)
            palette.setColor(QPalette::WindowText, QColor(255, 255, 0));
        else
            palette.setColor(QPalette::WindowText, QColor(255, 0, 0));

        label->setPalette(palette);
    }

    for (int i = 0; i < 4; i++)
    {
        QLabel *label = RobotControllerGUI::GetInstance().GetVescInfo(i, 1);
        QPalette palette = label->palette();

        if (canData.motorVoltage[i] > VOLT_WARN)
            palette.setColor(QPalette::WindowText, QColor(0, 255, 0));
        else if (canData.motorCurrent[i] > VOLT_RED)
            palette.setColor(QPalette::WindowText, QColor(255, 255, 0));
        else
            palette.setColor(QPalette::WindowText, QColor(255, 0, 0));

        label->setPalette(palette);
    }

    for (int i = 0; i < 4; i++)
    {
        QLabel *label = RobotControllerGUI::GetInstance().GetVescInfo(i, 3);
        QPalette palette = label->palette();

        if (canData.escFETTemp[i] < TEMP_WARN)
            palette.setColor(QPalette::WindowText, QColor(0, 255, 0));
        else if (canData.escFETTemp[i] < TEMP_RED)
            palette.setColor(QPalette::WindowText, QColor(255, 255, 0));
        else
            palette.setColor(QPalette::WindowText, QColor(255, 0, 0));

        label->setPalette(palette);
    }
}

void IMUWidget::_UpdateIMUInfos()
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

    // lock the mutex
    _matMutex.lock();
    // copy the mat
    mat.copyTo(_mat);
    // unlock the mutex
    _matMutex.unlock();
}

/**
 * Draws the new drawing image
 */
void IMUWidget::Update()
{
    _UpdateIMUInfos();
    _UpdateVESCInfos();
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
