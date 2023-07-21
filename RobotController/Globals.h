#ifndef GLOBALS_H
#define GLOBALS_H
#include <QString>

#include <opencv2/opencv.hpp>

#define WIDTH 360
#define HEIGHT 360

extern cv::Mat drawingImage;
extern QString SAVE_FILE_NAME;

// TODO: this should not be in globals
struct RobotIMUData
{
    cv::Point2f velocity;
    double angle;
};

extern RobotIMUData robotIMUData;


// #define ENABLE_TIMERS

#ifdef ENABLE_TIMERS
    #define TIMER_INIT Clock c;
    #define TIMER_START c.markStart();
    #define TIMER_PRINT(msg) std::cout << msg << " time: " << c.getElapsedTime() << std::endl;
#else
    #define TIMER_INIT 
    #define TIMER_START 
    #define TIMER_PRINT(msg)
#endif

#endif