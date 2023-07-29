#pragma once
#include <QString>

#include <opencv2/opencv.hpp>
#include <mutex>

#define WIDTH 720
#define HEIGHT 720

// the drawing image is the main image that is displayed on the screen
// it is set in the vision code and displayed in the main window
extern cv::Mat DRAWING_IMAGE;
extern std::mutex DRAWING_IMAGE_MUTEX;

extern QString SAVE_FILE_NAME;

// TODO: this should not be in globals
struct RobotIMUData
{
    cv::Point2f velocity;
    double angle;
};

extern RobotIMUData robotIMUData;

// uncomment below to enable timer printing
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
