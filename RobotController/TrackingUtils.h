#ifndef TRACKINGUTILS_H
#define TRACKINGUTILS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/video/video.hpp>

class TrackingUtils
{
private:
    TrackingUtils();

public:
    static cv::Point2f RotatePoint(cv::Point2f, cv::Point2f, double);
    static cv::Point2f ScalePoint(cv::Point2f, cv::Point2f, double);
    static double Distance(cv::Point2f, cv::Point2f);
    static double AngleBetweenPoints(cv::Point2f, cv::Point2f);
    static float Round(float, float);
    static double AngleWrap(double);
};

#endif