#include "MathUtils.h"

double AngleBetweenPoints(double x1, double y1, double x2, double y2)
{
    return std::atan2(y2 - y1, x2 - x1);
}

double angle_wrap(double angle_rad)
{
    angle_rad = fmod(angle_rad, 2 * M_PI);
    if (angle_rad <= -M_PI)
    {
        angle_rad += 2 * M_PI;
    }
    else if (angle_rad > M_PI)
    {
        angle_rad -= 2 * M_PI;
    }
    return angle_rad;
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

float distance(cv::Point2f point1, cv::Point2f point2 )
{
    float dx = point1.x - point2.x;
    float dy = point1.y - point2.y;

    return std::sqrt(dx * dx + dy * dy);
}

// A function to calculate the tangent points of a circle given the center, radius and an external point
void CalculateTangentPoints(const cv::Point2f& center, float radius, const cv::Point2f& externalPoint, cv::Point2f& tangent1, cv::Point2f& tangent2)
{
    // Calculate the distance between the external point and the center
    float dist = cv::norm(center - externalPoint);
    if (dist < radius)
    {
        // std::cerr << "ERROR CalculateTangentPoints: The external point is inside the circle" << std::endl;
        return;
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

cv::Point2f InterpolatePoints(const cv::Point2f& p1, const cv::Point2f& p2, double ratio)
{
    cv::Point2f diff = p2 - p1;
    return p1 + diff * ratio;
}

Angle InterpolateAngles(Angle a1, Angle a2, double ratio)
{
    // Find the difference between the angles and make sure it's the shortest path
    double diff = angle_wrap(a2 - a1);

    // Interpolate using the ratio
    return Angle(a1 + diff * ratio);
}

std::vector<cv::Point2f> CirclesIntersect(cv::Point2f center1, float r1, cv::Point2f center2, float r2)
{
    std::vector<cv::Point2f> intersections;

    float x1 = center1.x, y1 = center1.y;
    float x2 = center2.x, y2 = center2.y;

    // Compute the distance between the centers of the circles
    float dist = cv::norm(center1 - center2);

    // Check if there's no solution
    if (dist > r1 + r2 || dist < std::abs(r1 - r2) || (dist == 0 && r1 == r2))
    {
        return intersections; // Return an empty vector
    }

    // Compute a, b, and c
    float a = (r1 * r1 - r2 * r2 + dist * dist) / (2 * dist);
    float h = std::sqrt(r1 * r1 - a * a);
    float cx2 = x1 + a * (x2 - x1) / dist;
    float cy2 = y1 + a * (y2 - y1) / dist;

    // Compute the intersection points
    float ix1 = cx2 + h * (y2 - y1) / dist;
    float iy1 = cy2 - h * (x2 - x1) / dist;
    float ix2 = cx2 - h * (y2 - y1) / dist;
    float iy2 = cy2 + h * (x2 - x1) / dist;

    intersections.emplace_back(ix1, iy1);
    intersections.emplace_back(ix2, iy2);

    return intersections;
}


/**
 * Uses the error between a currentPos and targetPos
 * and returns a power to drive towards the target at (with 2 thresholds)
 *
 * @param error The target position
 * @param threshold1 The first threshold (full power)
 * @param threshold2 The second threshold (min power)
 * @param minPower The minimum power to drive at
 */
double DoubleThreshToTarget(double error,
                            double threshold1, double threshold2,
                            double minPower, double maxPower)

{
    double distance = std::abs(error);
    double ret = 0;

    if (distance >= threshold1)
    {
        // Move towards the target with full power
        ret = maxPower;
    }
    else if (distance < threshold1 && distance > threshold2)
    {
        // Scale linearly from maxPower to minPower
        ret = ((distance - threshold2) / (threshold1 - threshold2)) * (maxPower - minPower) + minPower;
    }
    else
    {
        // Scale linearly from minPower to 0
        ret = (distance / threshold2) * minPower;
    }

    // invert if we need to
    if (error < 0)
    {
        ret *= -1;
    }

    return ret;
}
