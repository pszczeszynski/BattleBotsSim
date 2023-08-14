#include "Globals.h"

QString SAVE_FILE_NAME = "RobotConfig.txt";

cv::Mat P_DRAWING_IMAGE = cv::Mat(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3));
bool CAN_DRAW = true;

std::mutex DRAWING_IMAGE_MUTEX;
