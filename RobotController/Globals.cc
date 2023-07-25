#include "Globals.h"

cv::Mat DRAWING_IMAGE = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
QString SAVE_FILE_NAME = "RobotConfig.txt";


RobotIMUData robotIMUData = {cv::Point2f(0, 0), 0};


double movement = 0;
double turn = 0;
bool wDown = false;
bool aDown = false;
bool sDown = false;
bool dDown = false;
