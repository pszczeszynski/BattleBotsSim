#pragma once
// #include <QString>

#include <mutex>
#include <opencv2/opencv.hpp>


#define WIDTH 720
#define HEIGHT 720

extern bool RESET_IMU;

extern bool EDITING_HEU;
extern bool EDITING_BLOB;
extern bool EDITING_OPENCV;

// the drawing image is the main image that is displayed on the screen
// it is set in the vision code and displayed in the main window
extern std::string SAVE_FILE_NAME;

extern double opponentRotationSim;  // just for hacking around with
extern double opponentRotationVelSim;
extern cv::Point2f robotPosSim;
extern cv::Point2f opponentPosSim;
extern cv::Point2f robotVelSim;
extern cv::Point2f opponentVelSim;
extern double simReceiveLastTime;

// uncomment below to enable timer printing
// #define ENABLE_TIMERS

#ifdef ENABLE_TIMERS
#define TIMER_INIT Clock c;
#define TIMER_START c.markStart();
#define TIMER_PRINT(msg) \
  std::cout << msg << " time: " << c.getElapsedTime() << std::endl;
#else
#define TIMER_INIT
#define TIMER_START
#define TIMER_PRINT(msg)
#endif
