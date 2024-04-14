#include "BruteForceRotation.h"

/////////////// ROTATION CALCULATOR ///////////////
RotationCalculator::RotationCalculator()
{
    foregroundAngle = 0;
}

double RotationCalculator::ComputeRotation(cv::Mat currForeground, cv::Point2f robotPos)
{
    const double CROP_SIZE = 128;
    // get the foreground
    cv::Mat currForeground = fieldImage(cv::Rect(robotPos.x - CROP_SIZE / 2, robotPos.y - CROP_SIZE / 2, CROP_SIZE, CROP_SIZE));

    // iterate from 0 to 360 degrees
    double bestAngle = 0;
    double bestScore = std::numeric_limits<double>::max();

    // iterate through all angles
    for (int i = 0; i < 360; i++)
    {
        // rotate the saved foreground
        cv::Mat rotatedForeground;
        cv::warpAffine(foreground, rotatedForeground,
                       cv::getRotationMatrix2D(cv::Point2f(CROP_SIZE / 2, CROP_SIZE / 2), i, 1.0),
                       cv::Size(CROP_SIZE, CROP_SIZE));

        // compute the score via subtraction
        cv::Mat diff;
        cv::absdiff(currForeground, rotatedForeground, diff);
        double score = cv::sum(diff)[0];

        // check if this is the best score
        if (score < bestScore)
        {
            bestScore = score;
            bestAngle = i;
        }
    }

    // return the best angle in radians
    return angle_wrap(bestAngle * TO_RAD + foregroundAngle);
}

void RotationCalculator::SetForeground(cv::Mat newForeground, double angle)
{
    foreground = newForeground;
    foregroundAngle = angle;
}

/////////////// BRUTE FORCE ROTATION ///////////////
BruteForceRotation::BruteForceRotation()
{
    _opponentRotationCalculator = RotationCalculator();
}

void BruteForceRotation::SetAngle(double newAngle, bool opponentRobot)
{
    if (opponentRobot)
    {
        // get 
        _opponentRotationCalculator.SetForeground(_opponentForeground, newAngle);
    }
    else
    {
        std::cerr << "BruteForceRotation::SetAngle: Setting angle for own robot is not supported" << std::endl;
    }
}

