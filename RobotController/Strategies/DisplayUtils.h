#pragma once

#include <opencv2/core.hpp>
#include "../SafeDrawing.h"

class DisplayUtils {

public:

    static void displayPoints(std::vector<cv::Point2f>& points, cv::Scalar startColor, cv::Scalar endColor, int radius);
    static void displayPath(std::vector<cv::Point2f>& pathPoints, cv::Scalar startColor, cv::Scalar endColor, int thick);



private:

};