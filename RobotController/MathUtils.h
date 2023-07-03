#pragma once
#include <cmath>
#include <opencv2/opencv.hpp>

#define M_PI 3.141592653589793238
#define TO_RAD (M_PI / 180.0)
#define TO_DEG (180.0 / M_PI)

// TODO: move this to a better place
double angle_between_points(double x1, double y1, double x2, double y2);

double angle_wrap(double angle);

cv::Point3f rotate3dPoint(cv::Point3f p, double angle_rad);

void calculateTangentPoints(const cv::Point2f& center, float radius, const cv::Point2f& externalPoint, cv::Point2f& tangent1, cv::Point2f& tangent2);

class Point2f
{
public:
    double x;
    double y;
    Point2f(double x, double y)
        : x(x), y(y)
    {
    }

    // define addition operator
    Point2f operator+(const Point2f& p) const
    {
        return Point2f(x + p.x, y + p.y);
    }

    // define multiplication operator
    Point2f operator*(const double& d) const
    {
        return Point2f(x * d, y * d);
    }

    // define subtraction operator
    Point2f operator-(const Point2f& p) const
    {
        return Point2f(x - p.x, y - p.y);
    }

    // define division operator
    Point2f operator/(const double& d) const
    {
        return Point2f(x / d, y / d);
    }
};

double norm(Point2f p);

cv::Point2f rotate_point(cv::Point2f p, double angle);
