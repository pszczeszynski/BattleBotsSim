#include "Globals.h"

cv::Mat P_DRAWING_IMAGE = cv::Mat(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3));

std::mutex DRAWING_IMAGE_MUTEX;

bool RESET_IMU = false;

float playback_speed = 1.0f;
bool playback_play = true;
bool playback_pause = false;
bool playback_goback = false;
bool playback_restart = false;
std::string playback_file = "./Recordings/OrbitronVsDisarray.avi";
bool playback_file_changed = true;
float playback_video_length_s = 0;
float playback_video_pos_s = 0;


bool EDITING_HEU = true;
bool EDITING_BLOB = true;

// just for hacking around with
double opponentRotationSim = 0;
cv::Point2f robotPosSim = cv::Point2f{};
cv::Point2f opponentPosSim = cv::Point2f{};