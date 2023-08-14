#pragma once

// IMUWiget.h
// The imu widget is an opencv mat that visualizes the imu data
// There is a circle outline (filling up the mat) and a crosshair in the middle
// There is a small dot that shows the current acceleration as a 2d vector

#include <QLabel>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <mutex>

class IMUWidget : public QLabel
{
    Q_OBJECT

public:
    IMUWidget(QWidget *parent = nullptr);
    static IMUWidget& GetInstance();
    void SetMat(const cv::Mat& mat);

    void Update();

public slots:
    void Draw();

private:
    static IMUWidget* _instance;

    cv::Mat _mat;
    QTimer _drawTimer;
    std::mutex _matMutex;
};
