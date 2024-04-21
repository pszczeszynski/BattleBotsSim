#include "BruteForceRotation.h"

/////////////// ROTATION CALCULATOR ///////////////
RotationCalculator::RotationCalculator()
{
    _foregroundAngle = 0;
}

double RotationCalculator::ComputeRotation(cv::Mat currForeground, cv::Point2f robotPos)
{
    const int CROP_SIZE = 128;

    int x = std::max((int)robotPos.x - CROP_SIZE / 2, 0);
    int y = std::max((int)robotPos.y - CROP_SIZE / 2, 0);
    int w = std::min(720 - x, CROP_SIZE);
    int h = std::min(720 - y, CROP_SIZE);
    cv::Rect roi = cv::Rect(x, y, w, h);

    currForeground = currForeground(roi);    // iterate from 0 to 360 degrees
    cv::imwrite("./Odometry/BruteForceRotation/field.jpg", currForeground);
    double bestAngle = 0;
    double bestScore = std::numeric_limits<double>::max();

    // return 0;
    // iterate through all angles
    for (int i = 0; i < 360; i++)
    {
        // rotate the saved foreground
        cv::Mat rotatedForeground;
        cv::warpAffine(_foreground, rotatedForeground,
                       cv::getRotationMatrix2D(cv::Point2f(CROP_SIZE / 2, CROP_SIZE / 2), i, 1.0),
                       cv::Size(CROP_SIZE, CROP_SIZE));

        cv::imwrite("./Odometry/BruteForceRotation/" + std::to_string(i) + ".jpg", rotatedForeground);

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

    std::cout << "angle " << bestAngle << std::endl;

    // return the best angle in radians
    return angle_wrap(bestAngle * TO_RAD + _foregroundAngle);
}

void RotationCalculator::SetForeground(cv::Mat newForeground, double angle)
{
    _foreground = newForeground;
    _foregroundAngle = angle;
}

/////////////// BRUTE FORCE ROTATION ///////////////
BruteForceRotation::BruteForceRotation(ICameraReceiver *videoSource) : OdometryBase(videoSource)
{
    _opponentRotationCalculator = RotationCalculator();
    SetAngle(0, true);
    SetPosition(cv::Point2f(0, 0), true);
    cv::Mat foreground = cv::imread("./Odometry/BruteForceRotation/crop.jpg", cv::IMREAD_GRAYSCALE);
    _opponentRotationCalculator.SetForeground(foreground, 0);
}

void BruteForceRotation::SetAngle(double newAngle, bool opponentRobot)
{
    if (opponentRobot)
    {
        std::unique_lock<std::mutex> locker(_updateMutex);
        _currDataOpponent.robotAngle = Angle(newAngle);
        _currDataOpponent.robotAngleValid = true;
        _currDataOpponent.id++;
    }
}

void BruteForceRotation::SetPosition(cv::Point2f newPos, bool opponentRobot)
{
    if (opponentRobot)
    {
        std::unique_lock<std::mutex> locker(_updateMutex);
        _opponentPos = newPos;
    }
}

void BruteForceRotation::_ProcessNewFrame(cv::Mat currFrame, double frameTime)
{
    double resultAngle = _opponentRotationCalculator.ComputeRotation(currFrame, _opponentPos);
    SetAngle(resultAngle, true);
}
