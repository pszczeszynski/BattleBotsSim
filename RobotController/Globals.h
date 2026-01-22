#pragma once
// #include <QString>

#include <opencv2/opencv.hpp>


#define WIDTH 720
#define HEIGHT 720

extern bool RESET_IMU;

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
