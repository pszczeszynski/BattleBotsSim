#include "RobotTracker.h"
#include "MathUtils.h"
#include "OpponentProfile.h"

RobotTracker::RobotTracker(cv::Point2f initialPosition) :
    position{initialPosition},
    isValid{false}
{
}


cv::Point2f RobotTracker::getExtrapolatedPos()
{
    if (!isValid)
    {
        return cv::Point2f(0, 0);
    }

    double timeSinceLastUpdate = lastUpdate.getElapsedTime();
    return position + velocity * timeSinceLastUpdate;
}

/**
 * @brief getCostOfUpdating
 * @param blob - the motion blob to potentially update to
 * @return the cost of updating the position to the given position.
*/
double RobotTracker::getCostOfUpdating(MotionBlob& blob)
{
    if (!isValid)
    {
        return 0;
    }

    double timeSinceLastUpdate = lastUpdate.getElapsedTime();
    cv::Point2f expected_pos = position + velocity * timeSinceLastUpdate;
    double positionCost = norm(blob.center - expected_pos);

    positionCost /= 30.0;
    if (positionCost > 1)
    {
        positionCost = 1;
    }

    double areaCost = abs(blob.rect.area() - lastBlob.rect.area()) / (double) lastBlob.rect.area();

    if (areaCost > 1)
    {
        areaCost = 1;
    }

    // return the average of the two costs
    return (positionCost + areaCost) / 2.0;
}

/**
 * @brief update
 * Updates the position of the robot to the given position.
 * @param blob - the MotionBlob to update to
*/
void RobotTracker::update(MotionBlob& blob, cv::Mat& frame)
{
    velocity = (blob.center - position) / lastUpdate.getElapsedTime();
    position = blob.center;
    lastBlob = blob;

    lastUpdate.markStart();

    // crop frame around newPosition + save
    const int CROP_SIZE = 50;
    cv::Mat croppedFrame;
    cv::Rect crop = cv::Rect(position.x - CROP_SIZE / 2, position.y - CROP_SIZE / 2, CROP_SIZE, CROP_SIZE);
    if (crop.x < 0 || crop.y < 0 || crop.x + crop.width > frame.cols || crop.y + crop.height > frame.rows)
    {
        return;
    }
    croppedFrame = frame(crop);

    if (croppedFrameLast.empty())
    {
        croppedFrameLast = croppedFrame;
        angle = 0;
        return;
    }


    // check if space pressed
    if (cv::waitKey(1) == 32)
    {
        std::cout << "space" << std::endl;
        croppedFrameLast = croppedFrame;
    }

    angle = getRotationBetweenMats(croppedFrameLast, croppedFrame, position - cv::Point2f(crop.tl()));


    croppedFrameLast = croppedFrame;

    cv::imshow("cropped", croppedFrame);
    isValid = true;

}

/**
 * @brief getPosition
 * @return the current position of the robot.
*/
cv::Point2f RobotTracker::getPosition()
{
    return position;
}

double RobotTracker::getRotationBetweenMats(cv::Mat &img1, cv::Mat &img2, cv::Point2f center)
{
    cv::Mat gray1, gray2;
    cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);

    double bestScore = std::numeric_limits<double>::infinity();
    double bestAngle = 0.0;
    double step = 1; // rotation step in degrees
    // cv::Mat bestRotMat;

    
    double curr_angle_deg = angle * TO_DEG;
    for (double ang = curr_angle_deg - 90; ang < curr_angle_deg + 90; ang += step)
    {
        cv::Mat rotMat = cv::getRotationMatrix2D(center, ang, 1.0);
        cv::Mat rotated;
        cv::warpAffine(gray2, rotated, rotMat, gray2.size());

        cv::Mat diff;
        cv::absdiff(rotated, gray1, diff);

        double score = cv::sum(diff)[0];

        if (score < bestScore)
        {
            bestScore = score;
            bestAngle = ang;
            // bestRotMat = rotMat.clone();
        }
    }

    // lastRotMat = bestRotMat;

    return angle_wrap(bestAngle * TO_RAD);
}

double RobotTracker::getAngle()
{
    return angle;
}

void RobotTracker::invalidate()
{
    isValid = false;
}

cv::Point2f RobotTracker::getVelocity()
{
    return velocity;
}