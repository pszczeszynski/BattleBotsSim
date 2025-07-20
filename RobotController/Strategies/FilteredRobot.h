#include <cmath>
#include <opencv2/core.hpp>
#include "../Globals.h"

// kalman filterd robot data
class FilteredRobot
{
 public:
  FilteredRobot();
  // pathSpacing and pathLength are visual params
  FilteredRobot(float pathSpacing, float pathLength, float moveSpeed,
                float moveAccel, float turnSpeed, float turnAccel);

  void updateFilters(float deltaTime, cv::Point2f visionPos,
                     float visionTheta);  // run filter update to find new data
//   void updateFiltersOpp(float deltaTime, cv::Point2f visionPos,
//                         float visionTheta);  // uses path tangency

  float collideETA(
      FilteredRobot& opp, bool forward,
      float collisionRadius);  // estimated time to collide with a robot
  float moveETASim(float distance, float startVel, bool print);
  float pointETASim(cv::Point2f point, float lagTime, float turnCW,
                    float angleMargin, bool forward,
                    bool print);  // simulate estimated turn time to point
  float turnTimeMin(cv::Point2f point, float lagTime, float angleMargin,
                    bool forward,
                    bool print);  // minimum time of both turn directions

  void updatePath();
  void setPos(std::vector<float> pos);
  std::vector<std::vector<float>> constAccExtrap(float time);
  std::vector<std::vector<float>> constVelExtrap(float time);
  float angleTo(cv::Point2f point, bool forward);
  float distanceTo(cv::Point2f point);

  // get robot data
  cv::Point2f position();
  cv::Point2f moveVel();
  std::vector<float> getPosFiltered();
  std::vector<float> getVelFiltered();
  std::vector<float> getAccFiltered();
  float turnVel();
  float getMaxTurnSpeed();
  float moveAccel();
  float turnAccel();
  float tangentVel(bool forward);
  float angle(bool forward);
  std::vector<cv::Point2f> getPath();
//   void pathTangency(float deltaTime, float visionTheta);

 private:
  float fieldMax = WIDTH;
  float fieldMin = 0.0f;

  // each are 3 big, XYT
  std::vector<float> posFiltered;  // current pos
  std::vector<float> velFiltered;  // current vel
  std::vector<float> accFiltered;  // current accel

  // extra filtered for threshold use
  std::vector<float> velFilteredSlow;

  // tracks how far off the relative angle is from true heading based on path
  // tangency
//   float visionAngleOffset;

  // data about this robot
  float maxTurnSpeed;  // assumed turn speed when doing ETA calcs
  float maxTurnAccel;  // assumed turn accel when doing ETA calcs

  float maxMoveSpeed;  // fastest we can go
  float maxMoveAccel;  // fastest we can accel

  std::vector<cv::Point2f> path;  // tracks where robot has been
  float pathSpacing;  // how far apart are the points in the tracked path
  float pathLength;   // how long the total tracked path is

  int sign(float num);
  cv::Point2f clipInBounds(cv::Point2f point);
};