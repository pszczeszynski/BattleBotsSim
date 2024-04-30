#include "PathFinder.h"
#include "MathUtils.h"

PathFinder::PathFinder()
{
}

/**
 * @brief Returns a vector that the robot should move in to attack the opponent.
 */
cv::Point2f PathFinder::GetMotionVector(cv::Point2f opponentPosition, double opponentAngle, OpponentProfile &opponentProfile, cv::Point2f pos)
{
    // get vector
    double NORMALIZE_RADIUS = 100;

    // get the relative position of us to the opponent
    cv::Point2f relativeToOpponent = pos - opponentPosition;
    // rotate it by the opponent's angle
    cv::Point2f rotated = rotate_point(relativeToOpponent, -opponentAngle);

    // normalize
    cv::Point2f normalized = rotated / NORMALIZE_RADIUS;

    // return the best attack vector
    cv::Point2f directionNotRotated = opponentProfile.CalcDirectionNormalized(normalized);

    // rotate back
    cv::Point2f ret = rotate_point(directionNotRotated, opponentAngle);

    return ret;
}

cv::Point2f project(cv::Point2f vector, cv::Point2f onto)
{
    return onto * vector.dot(onto) / onto.dot(onto);
}

cv::Point2f getPerpendicularVector(const cv::Point2f &vector, const cv::Point2f &onto)
{
    cv::Point2f projection = project(vector, onto);
    cv::Point2f perpendicular = vector - projection;
    return perpendicular;
}

cv::Point2f PathFinder::GetMotionVectorConvex(cv::Point2f opponentPosition, double opponentAngle, OpponentProfile &opponentProfile, cv::Point2f startPos, cv::Mat &drawingImage)
{
    const double stepSize = 2;
    const double STOP_RADIUS = 10;
    const double STOP_LENGTH = 300;

    cv::Point2f startToOpponent = opponentPosition - startPos;
    cv::Point2f startToOpponentNormalized = startToOpponent / norm(startToOpponent);

    // start at current point
    cv::Point2f currPos = startPos;
    cv::Point2f mostConvexVector = GetMotionVector(opponentPosition, opponentAngle, opponentProfile, currPos);
    double mostConvexDot = 0; // startToOpponentNormalized.dot(mostConvexVector);

    double travelDistance = 0;

    // keep going until we are close enough to the opponent or we have traveled far enough
    while (norm(currPos - opponentPosition) > STOP_RADIUS && travelDistance < STOP_LENGTH)
    {
        cv::Point2f prevPos = currPos;
        // move in the direction of the motion vector
        currPos += GetMotionVector(opponentPosition, opponentAngle, opponentProfile, currPos) * stepSize;
        // update travel distance
        travelDistance += stepSize;

        // draw line
        cv::line(drawingImage, prevPos, currPos, cv::Scalar(0, 0, 255), 2);

        // compute vector from start to curr position
        cv::Point2f startToCurr = currPos - startPos;

        cv::Point2f startToCurrNormalized = startToCurr / norm(startToCurr);

        // project onto startToOpponent)
        cv::Point2f proj = getPerpendicularVector(startToOpponent, startToCurr);

        if (norm(proj) > mostConvexDot)
        {
            mostConvexDot = norm(proj);
            mostConvexVector = startToCurr;

            // draw line from projection to opponent
            cv::line(drawingImage, opponentPosition, opponentPosition - proj, cv::Scalar(255, 0, 0), 2);
        }
    }
    // draw line from start to most convex point
    cv::line(drawingImage, startPos, startPos + mostConvexVector, cv::Scalar(0, 255, 0), 2);

    // draw circle at most convex point
    cv::circle(drawingImage, startPos + mostConvexVector, 5, cv::Scalar(0, 255, 0), 2);

    return mostConvexVector;
}






















// Helper function to calculate the angle between two points
double angle(cv::Point2f p1, cv::Point2f p2)
{
    return std::atan2(p2.y - p1.y, p2.x - p1.x);
}

bool intersection(cv::Point2f center, double radius, cv::Point2f lineStart, cv::Point2f lineEnd, cv::Point2f& intersectionPoint)
{
    cv::Point2f d = lineEnd - lineStart;
    cv::Point2f f = lineStart - center;

    double a = d.dot(d);
    double b = 2 * f.dot(d);
    double c = f.dot(f) - radius * radius;

    // disciminant is b^2 - 4ac in the quadratic formula
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
    {
        return false; // No intersection
    }

    // Calculate the intersection points
    double sqrtDiscriminant = std::sqrt(discriminant);
    double t1 = (-b + sqrtDiscriminant) / (2 * a);
    double t2 = (-b - sqrtDiscriminant) / (2 * a);

    // Check if both intersection points are within the line segment
    bool isT1InRange = (t1 >= 0 && t1 <= 1);
    bool isT2InRange = (t2 >= 0 && t2 <= 1);

    // calculate where intersections are on the line
    cv::Point2f point1 = lineStart + t1 * d;
    cv::Point2f point2 = lineStart + t2 * d;

    if (isT1InRange && isT2InRange)
    {
        // Choose the intersection point closer to the end point
        double dist1 = cv::norm(lineEnd - point1);
        double dist2 = cv::norm(lineEnd - point2);
        intersectionPoint = (dist1 < dist2) ? point1 : point2;
        return true;
    }
    else if (isT1InRange)
    {
        intersectionPoint = point1;
        return true;
    }
    else if (isT2InRange)
    {
        intersectionPoint = point2;
        return true;
    }

    return false; // No valid intersection within the line segment
}

// Pure pursuit algorithm
cv::Point2f PathFinder::purePursuit(std::vector<cv::Point2f> &path, cv::Point2f robotPos, double lookaheadDistance, cv::Mat &image)
{
    double minDistance = std::numeric_limits<double>::max();
    cv::Point2f targetPoint;

    // Find the point on the path closest to the robot
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        cv::Point2f segmentStart = path[i];
        cv::Point2f segmentEnd = path[i + 1];

        cv::Point2f closestPoint;
        if (getClosestPointOnSegment(segmentStart, segmentEnd, robotPos, closestPoint))
        {
            double dist = cv::norm(robotPos - closestPoint);
            if (dist < minDistance)
            {
                minDistance = dist;
                targetPoint = closestPoint;
            }
        }
    }

    // Find the intersection point between the circle and the path
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        cv::Point2f segmentStart = path[i];
        cv::Point2f segmentEnd = path[i + 1];

        cv::Point2f intersectionPoint;
        if (intersection(robotPos, lookaheadDistance, segmentStart, segmentEnd, intersectionPoint))
        {
            targetPoint = intersectionPoint;
        }
    }

    // Draw path
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        cv::line(image, path[i], path[i + 1], cv::Scalar(0, 0, 255), 2);
    }

    // Draw lookahead circle
    cv::circle(image, robotPos, static_cast<int>(lookaheadDistance), cv::Scalar(255, 0, 0), 2);

    // Draw intersection point
    cv::circle(image, targetPoint, 5, cv::Scalar(0, 255, 0), -1);

    // Return the target point for visualization or further processing
    return targetPoint;
}
