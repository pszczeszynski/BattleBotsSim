#pragma once
#include <QString>

#include <opencv2/opencv.hpp>
#include <mutex>

#define WIDTH 720
#define HEIGHT 720

class ProtectedMat
{
public:
    // Construct by passing in a cv::Mat
    ProtectedMat(cv::Mat mat) : mat_(mat) {}

    template <typename Func>
    void with_lock(Func f)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        f(mat_);
    }

private:
    cv::Mat mat_;
    std::mutex mutex_;
};

// the drawing image is the main image that is displayed on the screen
// it is set in the vision code and displayed in the main window
extern ProtectedMat P_DRAWING_IMAGE;
#define SAFE_DRAW P_DRAWING_IMAGE.with_lock([&](cv::Mat& drawingImage) {
#define END_SAFE_DRAW });

extern QString SAVE_FILE_NAME;


extern bool shiftDown;
extern bool aDown;
extern bool pDown;
extern bool nearCorner;

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
