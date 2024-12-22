#ifndef SCOREREGION_H
#define SCOREREGION_H

#include <vector>
#include <iostream>
#include <cmath>
#include <opencv2/core.hpp>
#include "Line.h"

class ScoreRegion {
public:

    ScoreRegion();
    ScoreRegion(std::vector<Line> lines_);

    std::vector<Line> fieldLines(const cv::Point2f& position, const float& theta);

private:

    std::vector<Line> lines;

    cv::Point2f pointRobotToField(const cv::Point2f& point, const cv::Point2f& robotPos, const float& robotAngle);

};

#endif
