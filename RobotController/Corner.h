#ifndef CORNER_H
#define CORNER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "TrackingUtils.h"
// Corner.h

class Corner
{
public:
    Corner(cv::Point2f, cv::Point2f, cv::Point2f);
    double GetTravelDistLastFrame(bool shifted = false);
    double GetDeltaXLastFrame(bool shifted = false);
    double GetDeltaYLastFrame(bool shifted = false);
    void SetRotAndScale(double, double, int, int);
    cv::Point2f pi;
    cv::Point2f pl;
    cv::Point2f pf;

    // pf if there was no rotation or scaleing this frame
    cv::Point2f pf_shifted;

    bool isCrazy = false;
};

#endif
