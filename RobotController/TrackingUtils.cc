#include "TrackingUtils.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/video/video.hpp>
#include <math.h>
#include <map>
#include <algorithm>

using namespace std;
using namespace cv;

#define PI 3.14159265358979

cv::Point2f TrackingUtils::RotatePoint(cv::Point2f p, cv::Point2f center, double angle)
{
    double s = sin(angle);
    double c = cos(angle);

    // translate point back to origin
    p.x -= center.x;
    p.y -= center.y;

    // rotate point
    float xnew = p.x * c - p.y * s;
    float ynew = p.x * s + p.y * c;

    // translate point back
    p.x = xnew + center.x;
    p.y = ynew + center.y;
    return p;
}

cv::Point2f TrackingUtils::ScalePoint(cv::Point2f p, cv::Point2f center, double amount)
{
    double offsetX = p.x - center.x;
    double offsetY = p.y - center.y;
    offsetX *= amount;
    offsetY *= amount;
    return cv::Point2f(center.x + offsetX, center.y + offsetY);
}

float TrackingUtils::Round(float x, float resolution)
{
    return round(x / resolution) * resolution;
}

double TrackingUtils::AngleWrap(double angle)
{
    while (angle > PI)
    {
        angle -= 2 * PI;
    }

    while (angle < -PI)
    {
        angle += 2 * PI;
    }
    return angle;
}

double TrackingUtils::Distance(cv::Point2f pf, cv::Point2f p2)
{
    double deltaX = p2.x - pf.x;
    double deltaY = p2.y - pf.y;
    double distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
    return distance;
}

double TrackingUtils::AngleBetweenPoints(cv::Point2f pf, cv::Point2f p2)
{
    double deltaX = p2.x - pf.x;
    double deltaY = p2.y - pf.y;
    return atan2(deltaY, deltaX);
}


/**
 * Rotates a point around the y axis (x and z affected)
*/
cv::Point3f TrackingUtils::rotate_point(cv::Point3f p, double angle_rad)
{
    // Rotate around y axis
    float x1 = p.x * cos(angle_rad) + p.z * sin(angle_rad);
    float y1 = p.y;
    float z1 = -p.x * sin(angle_rad) + p.z * cos(angle_rad);

    return cv::Point3f(x1, y1, z1);
}