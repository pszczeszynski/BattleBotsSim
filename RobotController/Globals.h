#pragma once
// #include <QString>

#include <opencv2/opencv.hpp>
#include <mutex>

#define WIDTH 720
#define HEIGHT 720


extern bool CAN_DRAW;

extern bool EDITING_HEU;
extern bool EDITING_BLOB;

extern float playback_speed;
extern bool playback_play;
extern bool playback_goback;
extern bool playback_restart;
extern std::string playback_file;
extern bool playback_file_changed;

// the drawing image is the main image that is displayed on the screen
// it is set in the vision code and displayed in the main window
extern cv::Mat P_DRAWING_IMAGE;
extern std::mutex DRAWING_IMAGE_MUTEX;
#define SAFE_DRAW DRAWING_IMAGE_MUTEX.lock(); { cv::Mat& drawingImage = P_DRAWING_IMAGE;
#define END_SAFE_DRAW } DRAWING_IMAGE_MUTEX.unlock();


extern std::string SAVE_FILE_NAME;

extern double opponentRotationSim; // just for hacking around with
extern cv::Point2f robotPosSim;
extern cv::Point2f opponentPosSim;

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
