#include <cmath>
#include <opencv2/core.hpp>


// kalman filterd robot data
class FilteredRobot
{
public:

    FilteredRobot();
    FilteredRobot(float pathSpacing, float pathLength);
    

    
    // run filter update to find new data
    void updateFilters(float deltaTime, cv::Point2f visionPos, float visionTheta);
    void updatePath();

    // get robot data
    cv::Point2f pos();
    cv::Point2f moveVel();
    float angle();
    float turnVel();
    std::vector<cv::Point2f> getPath();

    

private:

    float fieldMax = 720.0f;
    float fieldMin = 0.0f;

    cv::Point2f posFiltered; // current position
    cv::Point2f moveVelFiltered; // current move velocity in x and y
    float angleFiltered; // current angle
    float turnVelFiltered; // current turn velocity

    std::vector<cv::Point2f> path; // tracks where robot has been
    float pathSpacing; // how far apart are the points in the tracked path
    float pathLength; // how long the total tracked path is




    float wrapAngle(float angle);
    cv::Point2f clipInBounds(cv::Point2f point);
};