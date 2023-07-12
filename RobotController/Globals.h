#ifndef GLOBALS_H
#define GLOBALS_H
#include <QString>

#include <opencv2/opencv.hpp>

#define WIDTH 720
#define HEIGHT 720

extern cv::Mat drawingImage;
extern QString SAVE_FILE_NAME;




// TODO: this should not be in globals
struct RobotIMUData
{
    cv::Point2f velocity;
    double angle;
};

extern RobotIMUData robotIMUData;

#endif