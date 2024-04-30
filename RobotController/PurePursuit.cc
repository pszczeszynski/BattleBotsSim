#include "PurePursuit.h"
#include "MathUtils.h"

/**
 * Returns the intersections of the circle starting at the robot position with the path
 * 
 * @param robotPos The position of the robot
 * @param pathPoints The points of the path
 * @param followRadius The radius of the circle to follow the path with
 * @return The intersections of the circle with the path
*/
std::vector<cv::Point2f> PurePursuit::followPath(cv::Point2f robotPos, const std::vector<cv::Point2f> &pathPoints, double followRadius)
{
    std::vector<cv::Point2f> intersections;

    // Iterate over each segment in the path
    for (size_t i = 0; i < pathPoints.size() - 1; ++i)
    {
        // Get the start and end points of the segment
        cv::Point2f segmentStart = pathPoints[i];
        cv::Point2f segmentEnd = pathPoints[i + 1];

        // if segmentEnd is closer to the robot than segmentStart, continue
        // This prevents the robot from following the path backwards
        if (cv::norm(segmentEnd - robotPos) < cv::norm(segmentStart - robotPos))
        {
            continue;
        }

        // Compute intersections of the circle with current path segment
        std::vector<cv::Point2f> segmentIntersections =
            CircleLineSegmentIntersect(robotPos, static_cast<float>(followRadius), segmentStart, segmentEnd);

        // Append the intersections to the list
        intersections.insert(intersections.end(), segmentIntersections.begin(), segmentIntersections.end());
    }

    // if there are more than 2 intersections, keep only the 2 closest ones
    if (intersections.size() > 2)
    {
        std::cerr << "Warning followPath: More than 2 intersections found" << std::endl;
        // Sort the intersections by distance to the robot
        std::sort(intersections.begin(), intersections.end(), [robotPos](const cv::Point2f &a, const cv::Point2f &b) {
            return cv::norm(a - robotPos) < cv::norm(b - robotPos);
        });

        // Keep only the two closest intersections
        intersections.resize(2);
    }

    // // if there are no intersections, find the closest point on the paths
    // if (intersections.size() == 0)
    // {
    //     std::cerr << "Warning followPath: No intersections found. Projecting to find closest point on path" << std::endl;
    //     // project onto the path
    //     cv::Point2f closestPoint;
    //     double minDistance = 100000000.0; // some large number

    //     // Iterate over each segment in the path
    //     for (size_t i = 0; i < pathPoints.size() - 1; ++i)
    //     {
    //         cv::Point2f segmentStart = pathPoints[i];
    //         cv::Point2f segmentEnd = pathPoints[i + 1];

    //         // Get the closest point on the segment
    //         cv::Point2f closestPoint;
    //         if (getClosestPointOnSegment(segmentStart, segmentEnd, robotPos, closestPoint))
    //         {
    //             // if this distance beats the current minimum, update the minimum
    //             double dist = cv::norm(robotPos - closestPoint);
    //             if (dist < minDistance)
    //             {
    //                 minDistance = dist;
    //                 closestPoint = closestPoint;
    //             }
    //         }
    //     }

    //     // Add the closest point to the intersections
    //     intersections.push_back(closestPoint);        
    // }

    // Return the intersections
    return intersections;
}

/**
 * Returns the point tangent to the given path, given the robot position and given angle
 *
 * @param pathPoints The points of the path
 * @param robot The position of the robot
 * @param opponentWeaponPos The center of the opponent's robot
 * @param outTangent1 The first tangent point
 * @param outTangent2 The second tangent point
 */
void PurePursuit::CalculateTangentPoints(std::vector<cv::Point2f> &pathPoints,
                                         cv::Point2f robot,
                                         cv::Point2f opponentWeaponPos,
                                         cv::Point2f &outTangent1,
                                         cv::Point2f &outTangent2)
{
    double biggestAngle = 0;
    double smallestAngle = 0;

    double angleRobotToCenter = atan2(opponentWeaponPos.y - robot.y, opponentWeaponPos.x - robot.x);

    for (cv::Point2f point : pathPoints)
    {
        double angleRobotToPoint = atan2(point.y - robot.y, point.x - robot.x);
        double diff = -angle_wrap(angleRobotToPoint - angleRobotToCenter);

        if (diff > biggestAngle)
        {
            biggestAngle = diff;
            outTangent1 = point;
        }

        if (diff < smallestAngle)
        {
            smallestAngle = diff;
            outTangent2 = point;
        }
    }
}


/**
 * Returns the angle (in radians) of the closest segment
 * 
 * @param pathPoints The points of the path
 * @param robotPos The position of the robot
 * @return The angle of the closest point on the path
*/
double PurePursuit::GetAngleOfClosestPathSegment(const std::vector<cv::Point2f> &pathPoints, const cv::Point2f &robotPos)
{
    // Find the closest point on the path
    int closestPointIndex = -1;
    double minDistance = 100000000.0; // some large number

    // Iterate over each segment in the path
    for (size_t i = 0; i < pathPoints.size() - 1; ++i)
    {
        cv::Point2f segmentStart = pathPoints[i];
        cv::Point2f segmentEnd = pathPoints[i + 1];
        cv::Point2f midPoint = (segmentStart + segmentEnd) / 2;

        // if this distance beats the current minimum, update the minimum
        double dist = cv::norm(robotPos - midPoint);
        if (dist < minDistance)
        {
            minDistance = dist;
            closestPointIndex = i;
        }
    }

    if (closestPointIndex == -1)
    {
        std::cerr << "ERROR GetAngleOfClosestPathSegment: No closest point found" << std::endl;
        return 0;
    }

    cv::Point2f closestPoint = pathPoints[closestPointIndex];
    cv::Point2f nextPoint = pathPoints[closestPointIndex + 1];

    return atan2(nextPoint.y - closestPoint.y, nextPoint.x - closestPoint.x);
}