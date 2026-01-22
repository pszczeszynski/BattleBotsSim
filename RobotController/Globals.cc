#include "Globals.h"

bool RESET_IMU = false;

bool EDITING_HEU = true;
bool EDITING_BLOB = true;
bool EDITING_OPENCV = true;

// just for hacking around with
double opponentRotationSim = 0;
double opponentRotationVelSim = 0;
cv::Point2f robotPosSim = cv::Point2f{};
cv::Point2f robotVelSim = cv::Point2f{};
cv::Point2f opponentPosSim = cv::Point2f{};
cv::Point2f opponentVelSim = cv::Point2f{};
double simReceiveLastTime = 0;