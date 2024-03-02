#include "SelfRighter.h"
// cv
#include <opencv2/imgproc.hpp>
SelfRighter::SelfRighter()
{

}


/**
 * @brief Times the self righter and reduces power towards the limits
*/
void SelfRighter::Move(float inputPower, DriveCommand& command, cv::Mat& drawingImage)
{
    command.selfRighterPower = inputPower;

    // if the input power isn't 0, draw the self righter power
    if (abs(command.selfRighterPower) > 0)
    {
        cv::putText(drawingImage, "SELF RIGHT ACTIVATED!", cv::Point(drawingImage.cols * 0.8, drawingImage.rows * 0.8), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
    }
}