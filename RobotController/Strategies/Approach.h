#pragma once
#include <opencv2/core.hpp>


class Approach {

public:

    Approach();
    Approach(cv::Point2f center, float endAngle, float collisionRad, bool CW);


    std::vector<cv::Point2f> getCurvePoints();
    cv::Point2f followCurve(cv::Point2f currPosition, float ppRad, bool display);
    cv::Point2f getCenter();
    float radiusEquation(float offsetAngle);
    float curvatureHere(float offsetAngle, bool display);
    float offsetToPoint(cv::Point2f point);
    cv::Point2f closestPoint(cv::Point2f point, float advance);
    float absPathAngleHere(float offsetAngle);
    float wrapInSweep(float angle);
    bool getCW();
    bool outsideCurve(cv::Point2f point);
    cv::Point2f tangentPoint(cv::Point2f point);
    float getEndAngle();




private:

    cv::Point2f center; // center point of radial function
    std::vector<cv::Point2f> curvePoints; // points that actually define the curve

    float endAngle; // angle at which curve collides with center
    float collisionRad; // rad where curve points start since we collide at that rad
    bool CW; // is this curve for CW or CCW driving direction
    float sweepRange; // what angle the curve is defined for



    void generateCurve();
    cv::Point2f ppPoint(cv::Point2f currPosition, float ppRad);
    float angle(cv::Point2f point1, cv::Point2f point2);

};