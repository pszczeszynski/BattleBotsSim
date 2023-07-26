#include "Globals.h"

cv::Mat DRAWING_IMAGE = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
std::mutex DRAWING_IMAGE_MUTEX; // protects access to DRAWING_IMAGE
QString SAVE_FILE_NAME = "RobotConfig.txt";


RobotIMUData robotIMUData = {cv::Point2f(0, 0), 0};
