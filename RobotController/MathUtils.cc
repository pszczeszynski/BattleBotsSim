#include "MathUtils.h"

double angle_between_points(double x1, double y1, double x2, double y2)
{
    return std::atan2(y2 - y1, x2 - x1);
}

double angle_wrap(double angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle <= -M_PI)
    {
        angle += 2 * M_PI;
    }
    else if (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    return angle;
}

double norm(Point2f p)
{
    return std::sqrt(p.x * p.x + p.y * p.y);
}

cv::Point2f rotate_point(cv::Point2f p, double angle)
{
    return cv::Point2f(p.x * std::cos(angle) - p.y * std::sin(angle), p.x * std::sin(angle) + p.y * std::cos(angle));
}