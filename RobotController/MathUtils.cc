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

cv::Point3f rotate3dPoint(cv::Point3f p, double angle_rad)
{
    p.y = p.y * cos(angle_rad) - p.z * sin(angle_rad);
    p.z = p.y * sin(angle_rad) + p.z * cos(angle_rad);
    return p;
}

// A function to calculate the tangent points of a circle given the center, radius and an external point
void calculateTangentPoints(const cv::Point2f& center, float radius, const cv::Point2f& externalPoint, cv::Point2f& tangent1, cv::Point2f& tangent2)
{
    // Calculate the distance between the external point and the center
    float dist = cv::norm(center - externalPoint);
    if (dist < radius)
    {
        std::cerr << "ERROR calculateTangentPoints: The external point is inside the circle" << std::endl;
    }

    // Calculate the angle for the two tangent lines
    float angle = acos(radius / dist);
    float centerToExternalPointAngle = atan2(externalPoint.y - center.y, externalPoint.x - center.x);

    // Calculate the angles for the two tangent points
    float angle1 = centerToExternalPointAngle + angle;
    float angle2 = centerToExternalPointAngle - angle;

    // Calculate the tangent points using the angles and the radius
    tangent1 = cv::Point2f(center.x + radius * cos(angle1), center.y + radius * sin(angle1));
    tangent2 = cv::Point2f(center.x + radius * cos(angle2), center.y + radius * sin(angle2));
}
