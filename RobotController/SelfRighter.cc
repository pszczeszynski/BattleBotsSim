#include "SelfRighter.h"
// for min and max
#include <algorithm>
// put text
#include <opencv2/imgproc.hpp>

SelfRighter::SelfRighter() :
    _positionGuessPercent(0)
{

}

#define LIMITS_POWER_PERCENT 1.0

/**
 * @brief Times the self righter and reduces power towards the limits
*/
void SelfRighter::Move(float inputPower, DriveCommand& command, cv::Mat& drawingImage)
{
    double deltaTimeSeconds = _updateClock.getElapsedTime();

    // increment the guessed position linearly with input power and time
    _positionGuessPercent += inputPower * deltaTimeSeconds / RANGE_SECONDS_FULL_POWER;

    _updateClock.markStart();

    // default to the input power
    float outPower = inputPower;

    // if headed past the end of the range
    if (_positionGuessPercent > 0.95 && outPower > 0 ||
        _positionGuessPercent < 0.05 && outPower < 0)
    {
        // scale down the power
        outPower *= LIMITS_POWER_PERCENT;

        // give the driver a warning
        cv::putText(drawingImage, "SELFRIGHTER LIMIT!", cv::Point(drawingImage.cols * 0.8, drawingImage.rows * 0.8), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
    }

    // clamp the position guess
    _positionGuessPercent = std::min(1.0, _positionGuessPercent);
    _positionGuessPercent = std::max(0.0, _positionGuessPercent);

    // apply the power
    command.selfRighterPower = outPower;
}