#include "Globals.h"

cv::Mat drawingImage = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
QString SAVE_FILE_NAME = "RobotConfig.txt";


RobotIMUData robotIMUData = {cv::Point2f(0, 0), 0};