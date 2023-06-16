#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

struct MotionVector
{
    cv::Point2f position;
    cv::Point2f direction;
};


// opponents are represented as a collection of charges radially
class OpponentProfile
{
public:
    OpponentProfile();

    cv::Point2f CalcDirection(cv::Point2f p);
    cv::Point2f CalcDirectionNormalized(cv::Point2f p);

    void DrawGraphic(cv::Mat& drawing_image);

    std::vector<MotionVector> motionVectors;

    void AddVector(cv::Point2f position, cv::Point2f direction);
};
