#pragma once
#include <opencv2/core.hpp>


class Approach {

public:

    Approach();
    Approach(cv::Point2f center, float endAngle, float collisionRad, bool CW);


    std::vector<cv::Point2f> getCurvePoints();
    cv::Point2f followCurve(cv::Point2f currPosition, float ppRad, bool display);




private:

    cv::Point2f center; // center point of radial function
    std::vector<cv::Point2f> curvePoints; // points that actually define the curve

    float endAngle; // angle at which curve collides with center
    float collisionRad; // rad where curve points start since we collide at that rad
    bool CW; // is this curve for CW or CCW driving direction
    float sweepRange; // what angle the curve is defined for



    void generateCurve();
    float radiusEquation(float offsetAngle);
    cv::Point2f ppPoint(cv::Point2f currPosition, float ppRad);
    float angle(cv::Point2f point1, cv::Point2f point2);

};