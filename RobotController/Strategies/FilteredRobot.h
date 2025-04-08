#include <cmath>
#include <opencv2/core.hpp>


// kalman filterd robot data
class FilteredRobot
{
public:

    FilteredRobot();
    FilteredRobot(float pathSpacing, float pathLength, float turnSpeed, float turnAccel);
    
    
    void updateFilters(float deltaTime, cv::Point2f visionPos, float visionTheta); // run filter update to find new data
    void updateFiltersOpp(float deltaTime, cv::Point2f visionPos, float visionTheta); // uses path tangency


    float collideETA(FilteredRobot opp); // estimated time to collide with a robot
    float pointETA(cv::Point2f point, float angleMargin1, float angleMargin2, bool CW); // estimated time to face towards a point
    float timeToTurnToAngle(float angle, bool turnCW); // estimated time to turn to an angle given a certain turn direction


    void updatePath();
    void setPos(std::vector<float> pos);
    std::vector<std::vector<float>> constAccExtrap(float time);
    float angleTo(cv::Point2f point);

    // get robot data
    cv::Point2f position();
    cv::Point2f moveVel();
    float angle();
    float turnVel();
    float turnAccel();
    std::vector<cv::Point2f> getPath();
    void pathTangency(float deltaTime, float visionTheta);

    

private:

    float fieldMax = 720.0f;
    float fieldMin = 0.0f;


    // each are 3 big, XYT
    std::vector<float> posFiltered; // current pos
    std::vector<float> velFiltered; // current vel
    std::vector<float> accFiltered; // current accel

    // extra filtered for threshold use
    std::vector<float> velFilteredSlow;


    // tracks how far off the relative angle is from true heading based on path tangency
    float visionAngleOffset;


    // data about this robot
    float maxTurnSpeed; // assumed turn speed when doing ETA calcs
    float maxTurnAccel; // assumed turn accel when doing ETA calcs


    std::vector<cv::Point2f> path; // tracks where robot has been
    float pathSpacing; // how far apart are the points in the tracked path
    float pathLength; // how long the total tracked path is


    float wrapAngle(float angle);
    cv::Point2f clipInBounds(cv::Point2f point);
};