#include "OpponentProfile.h"
#include "MathUtils.h"
#include <iostream>
#include "RobotOdometry.h"
#include <algorithm>

OpponentProfile::OpponentProfile()
{
}

const int GRAPHIC_SIZE = 400;
const int GRAPHIC_RADIUS = GRAPHIC_SIZE / 2;
OpponentProfile *editing = nullptr;

cv::Point2f OpponentProfile::CalcDirection(cv::Point2f p)
{
    // returns weighted average of all the vectors, weighted by distance (closer = more important)
    cv::Point2f direction = cv::Point2f(0, 0);

    for (MotionVector vector : motionVectors)
    {
        // calculate weight
        double distance = norm(vector.position - p);
        double weight = distance > 0.0 ? 1.0 / pow(distance, 2) : 100000;
        direction += vector.direction * weight;
    }

    // divide by the total weight
    if (norm(direction) > 0.0)
    {
        direction /= norm(direction);
    }


    return direction;
}

cv::Point2f OpponentProfile::CalcDirectionNormalized(cv::Point2f p)
{
    cv::Point2f graphicCoords = ((p + cv::Point2f(1, 1)) / 2) * GRAPHIC_SIZE; 
    cv::Point2f direction = CalcDirection(graphicCoords);

    // but now, if we are outside the sphere, let's make it just go towards the center
    if (norm(p) > 1)
    {
        double ADDITIONAL_RADIUS_BLEND_LENGTH = 1;

        // blend slowly towards -p => takes another radius to go to point
        double blend = std::min((norm(p) - 1) / ADDITIONAL_RADIUS_BLEND_LENGTH, 1.0);

        
        direction = direction * (1 - blend) + (-p / norm(p)) * blend;
    }
    return direction;
}

void OpponentProfile::AddVector(cv::Point2f position, cv::Point2f direction)
{
    // normalize direction
    if (norm(direction) > 0.0)
    {
        direction /= norm(direction);
    }
    else
    {
        std::cerr << "ERROR: can't add zero length vector to opponent profile" << std::endl;
        return;
    }

    motionVectors.push_back(MotionVector{position, direction});

    // add symmetric vector
    motionVectors.push_back(MotionVector{cv::Point2f(position.x, GRAPHIC_SIZE - position.y), cv::Point2f(direction.x, -direction.y)});
}

/**
 * First click sets the origin, second click sets the end point
 */
void mouseCallback(int event, int x, int y, int flags, void *userdata)
{
    static cv::Point2f lastMouseDownPosition = cv::Point2f(0, 0);

    if (event == cv::EVENT_LBUTTONDOWN)
    {
        if (norm(lastMouseDownPosition) > 0)
        {
            editing->AddVector(lastMouseDownPosition, cv::Point2f(x, y) - lastMouseDownPosition);
            lastMouseDownPosition = cv::Point2f(0, 0);
        }
        else
        {
            lastMouseDownPosition = cv::Point2f(x, y);
        }
    }

    // undo with right click
    if (event == cv::EVENT_RBUTTONDOWN)
    {
        // remove last element (but symmetrical vector too)
        if (editing->motionVectors.size() > 0)
        {
            editing->motionVectors.pop_back();
            editing->motionVectors.pop_back();
        }
    }
}

void DrawArrow(cv::Mat& drawing_image, cv::Point2f start, cv::Point2f end, cv::Scalar color)
{
    // Draw a line from start to end
    cv::line(drawing_image, start, end, color, 2);

    // Calculate the angle between the line and the x-axis
    double angle = atan2(start.y - end.y, start.x - end.x);

    // Define the length and angle of the arrowhead
    double arrowheadLength = 10.0;
    double arrowheadAngle = CV_PI / 6;  // 30 degrees

    // Calculate the coordinates of the arrowhead points
    cv::Point2f arrowheadPoint1(end.x + arrowheadLength * cos(angle + arrowheadAngle),
                                end.y + arrowheadLength * sin(angle + arrowheadAngle));
    cv::Point2f arrowheadPoint2(end.x + arrowheadLength * cos(angle - arrowheadAngle),
                                end.y + arrowheadLength * sin(angle - arrowheadAngle));

    // Draw the two lines for the arrowhead
    cv::line(drawing_image, end, arrowheadPoint1, color, 2);
    cv::line(drawing_image, end, arrowheadPoint2, color, 2);
}

/**
 * Draws the opponent profile on the given image
 * @param drawing_image the image to draw the opponent profile on
 */
void OpponentProfile::DrawGraphic(cv::Mat &drawing_image)
{
    // draw horizontal line and vertical line
    cv::line(drawing_image, cv::Point2f(0, GRAPHIC_RADIUS), cv::Point2f(GRAPHIC_SIZE, GRAPHIC_RADIUS), cv::Scalar(255, 255, 255), 1);
    cv::line(drawing_image, cv::Point2f(GRAPHIC_RADIUS, 0), cv::Point2f(GRAPHIC_RADIUS, GRAPHIC_SIZE), cv::Scalar(255, 255, 255), 1);

    // draw circle have the radius of the graphic
    cv::circle(drawing_image, cv::Point2f(GRAPHIC_RADIUS, GRAPHIC_RADIUS), GRAPHIC_RADIUS / 2, cv::Scalar(255, 255, 255), 1);

    const cv::Point2f MID_POINT_IMAGE = cv::Point2f(GRAPHIC_RADIUS, GRAPHIC_RADIUS);

    const int step = 50;
    const int LINE_LENGTH = step / 2;

    // same as above but instead of lines, they are gradients originating from the center of the image
    for (int i = 0; i < drawing_image.cols; i += step)
    {
        for (int j = 0; j < drawing_image.rows; j += step)
        {
            cv::Point2f origin = cv::Point2f(i + step / 2, j + step / 2);
            cv::Point2f direction = CalcDirection(cv::Point2f(i, j));
            // draw line in the direction of the vector
            cv::Point2f end = cv::Point2f(i, j) + direction * LINE_LENGTH;

            cv::Scalar color = cv::Scalar(abs(direction.x) * 255, 0, abs(direction.y) * 255);

            DrawArrow(drawing_image, cv::Point2f(i, j), end, color);
            // // draw line
            // cv::line(drawing_image, cv::Point2f(i, j), end, color, 1);
        }
    }
    // draw green arrows for all existing vectors
    for (MotionVector vector : motionVectors)
    {
        cv::Point2f end = vector.position + vector.direction * LINE_LENGTH;
        DrawArrow(drawing_image, vector.position, end, cv::Scalar(0, 255, 0));
    }

    // the user could click to add a charge or right click to remove a charge

    editing = this;
    cv::namedWindow("Opponent Profile");
    // add undo gui

    cv::setMouseCallback("Opponent Profile", mouseCallback);

    cv::imshow("Opponent Profile", drawing_image);
    cv::waitKey(1);
}
