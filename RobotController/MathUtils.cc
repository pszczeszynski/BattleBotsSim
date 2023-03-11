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

/**
 * Rotates a point around the y axis (x and z affected)
*/
cv::Point3f rotate_point(cv::Point3f p, double angle_rad)
{
    // Rotate around y axis
    float x1 = p.x * cos(angle_rad) + p.z * sin(angle_rad);
    float y1 = p.y;
    float z1 = -p.x * sin(angle_rad) + p.z * cos(angle_rad);

    return cv::Point3f(x1, y1, z1);
}