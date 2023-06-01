#include "RobotTracker.h"
#include "MathUtils.h"

RobotTracker::RobotTracker(cv::Point2f initialPosition) :
    position{initialPosition},
    isValid{false}
{
}


/**
 * @brief getCostOfUpdating
 * @param newPosition - the new position of the robot.
 * @return the cost of updating the position to the given position.
*/
double RobotTracker::getCostOfUpdating(cv::Point2f newPosition)
{
    if (!isValid)
    {
        return 0;
    }

    return norm(newPosition - position);
}

/**
 * @brief update
 * Updates the position of the robot to the given position.
 * @param newPosition - the new position of the robot.
*/
void RobotTracker::update(cv::Point2f newPosition, cv::Mat& frame)
{
    position = newPosition;
    velocity = (position - lastPosition) / lastUpdate.getElapsedTime();
    lastUpdate.markStart();
    isValid = true;

    // crop frame around newPosition + save
    const int CROP_SIZE = 20;
    cv::Mat croppedFrame;
    cv::Rect crop = cv::Rect(newPosition.x - CROP_SIZE / 2, newPosition.y - CROP_SIZE / 2, CROP_SIZE, CROP_SIZE);
    if (crop.x < 0 || crop.y < 0 || crop.x + crop.width > frame.cols || crop.y + crop.height > frame.rows)
    {
        return;
    }
    croppedFrame = frame(crop);

    if (croppedFrameLast.empty())
    {
        croppedFrameLast = croppedFrame;
        return;
    }
    angle += getRotationBetweenMats(croppedFrameLast, croppedFrame);
    angle = angle_wrap(angle);
    croppedFrameLast = croppedFrame;

    cv::imshow("cropped", croppedFrame);
}

/**
 * @brief getPosition
 * @return the current position of the robot.
*/
cv::Point2f RobotTracker::getPosition()
{
    return position;
}

double RobotTracker::getRotationBetweenMats(cv::Mat& img1, cv::Mat& img2) {
    cv::Mat gray1, gray2;
    cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);

    double bestScore = std::numeric_limits<double>::infinity();
    double bestAngle = 0.0;
    double step = 0.2; // rotation step in degrees

    for(double angle = -20; angle < 20; angle += step) {
        cv::Mat rotMat = cv::getRotationMatrix2D(cv::Point2f(gray1.cols/2, gray1.rows/2), angle, 1.0);
        cv::Mat rotated;
        cv::warpAffine(gray1, rotated, rotMat, gray1.size());

        cv::Mat diff;
        cv::absdiff(rotated, gray2, diff);
        double score = cv::sum(diff)[0];

        if(score < bestScore) {
            bestScore = score;
            bestAngle = angle;
        }
    }

    return angle_wrap(-bestAngle * TO_RAD);
}

double RobotTracker::getAngle()
{
    return angle;
}