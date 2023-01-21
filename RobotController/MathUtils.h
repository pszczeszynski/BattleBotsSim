#pragma once
#include <cmath>

#define TO_RAD (M_PI / 180.0)
#define TO_DEG (180.0 / M_PI)

// TODO: move this to a better place
double angle_between_points(double x1, double y1, double x2, double y2);

double angle_wrap(double angle);