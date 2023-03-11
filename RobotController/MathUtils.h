#pragma once
#include <cmath>
#include <opencv2/core.hpp>

#define M_PI 3.141592653589793238
#define TO_RAD (M_PI / 180.0)
#define TO_DEG (180.0 / M_PI)

// TODO: move this to a better place
double angle_between_points(double x1, double y1, double x2, double y2);

double angle_wrap(double angle);

cv::Point3f rotate_point(cv::Point3f p, double angle_rad);
