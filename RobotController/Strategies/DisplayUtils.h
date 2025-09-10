#pragma once

#include <opencv2/core.hpp>
#include "../SafeDrawing.h"
#include "Line.h"

class DisplayUtils {

public:

    static void displayPoints(std::vector<cv::Point2f>& points, cv::Scalar startColor, cv::Scalar endColor, int radius);
    static void displayPath(std::vector<cv::Point2f>& pathPoints, cv::Scalar startColor, cv::Scalar endColor, int thick);
    static void displayLines(std::vector<Line>& lines, cv::Scalar color);
    static void displayLinesIndices(std::vector<Line>& lines, std::vector<int> indices, cv::Scalar color);
    static void emote();


private:

};