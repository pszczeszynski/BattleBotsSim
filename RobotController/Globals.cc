#include "Globals.h"

QString SAVE_FILE_NAME = "RobotConfig.txt";

ProtectedMat P_DRAWING_IMAGE = ProtectedMat(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3));

bool shiftDown = false;
bool aDown = false;
bool pDown = false;
