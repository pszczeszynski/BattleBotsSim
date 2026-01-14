#pragma once

#include "../SafeDrawing.h"
#include <vector>
#include "Line.h"


class Field {


public:


    Field();
    std::vector<cv::Point2f>& getBoundPoints();
    std::vector<Line> getBoundLines();
    bool intersectsAnyBound(Line testLine);
    bool insideFieldBounds(cv::Point2f point);
    cv::Point2f closestBoundPoint(cv::Point2f point);
    cv::Point2f clipPointInBounds(cv::Point2f testPoint);
    void setBoundPoints(const std::vector<cv::Point2f>& points);
    void generateBoundLines();
    void resetBoundsToDefault();
    std::pair<float, int> Field::closestFromLineList(std::vector<Line> lineList, const cv::Point2f& point);



private:

    std::vector<cv::Point2f> boundPoints; // list of points that define the field bound lines
    std::vector<Line> boundLines; // list of lines that defines the outline of the field

    
};
