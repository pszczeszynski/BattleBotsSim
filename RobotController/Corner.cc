#include "Corner.h"

Corner::Corner(cv::Point2f PI, cv::Point2f PL, cv::Point2f PF) :
    pi(PI), pl(PL), pf(PF), pf_shifted(PF)
{

}

void Corner::SetRotAndScale(double scaleDelta, double angleDelta, int imageWidth, int imageHeight)
{
    pf_shifted = TrackingUtils::RotatePoint(pf, cv::Point(imageWidth / 2, imageHeight / 2), -angleDelta);
    pf_shifted = TrackingUtils::ScalePoint(pf_shifted, cv::Point(imageWidth / 2, imageHeight / 2), 1 / scaleDelta);
}

double Corner::GetTravelDistLastFrame(bool shifted)
{
    if (!shifted)
    {
        return TrackingUtils::Distance(pl, pf);
    }
    else
    {
        return TrackingUtils::Distance(pl, pf_shifted);
    }
}

/**
 * @brief Get the delta x between the last frame and the current frame
 * @param shifted true if we should take out the rotation and scaling effects
*/
double Corner::GetDeltaXLastFrame(bool shifted)
{
    if (!shifted)
    {
        return pf.x - pl.x;
    }
    else
    {
        return pf_shifted.x - pl.x;
    }
}

/**
 * @brief Get the delta y between the last frame and the current frame
 * @param shifted true if we should take out the rotation and scaling effects
*/
double Corner::GetDeltaYLastFrame(bool shifted)
{
    if (!shifted)
    {
        return pf.y - pl.y;
    }
    else
    {
        return pf_shifted.y - pl.y;
    }
}
