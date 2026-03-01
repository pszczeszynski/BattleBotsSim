#pragma once

#include <cmath>
#include <opencv2/core.hpp>
#include "../Globals.h"

// kalman filtered robot data
class FilteredRobot
{
 public:
  FilteredRobot();

  // pathSpacing and pathLength are visual params
  FilteredRobot(float pathSpacing, float pathLength, float moveSpeed,
                float moveAccel, float turnSpeed, float turnAccel, 
                float weaponAngleReach, float sizeRadius);

  // for virtual opp with zero speed
  FilteredRobot(cv::Point2f position, float sizeRadius);

  void updateFilters(float deltaTime, cv::Point2f visionPos, float visionTheta);
  void tuneModel(bool autoTune, std::vector<float> inputs, std::vector<float> trueOutputs);
  std::vector<float> updateModel(std::vector<float> inputs, std::vector<float>& model, bool writePos);


  float collideETASimple(cv::Point2f point, float pointSizeRadius, bool forward);
  float pointETAAccel(cv::Point2f target, bool forward, float margin);
  float moveETAAccel(float distance, float startingVel);
  

  bool colliding(FilteredRobot opp, float tolerance);
  bool facing(FilteredRobot opp, bool forward);
  bool facingPoint(cv::Point2f point, bool forward);
  float angleTo(cv::Point2f point, bool forward);
  float distanceTo(cv::Point2f point);
  float distanceToCollide(FilteredRobot opp);


  void updatePath();
  void setPos(std::vector<float> pos);
  void setVel(std::vector<float> vel);
  void setAccel(std::vector<float> acc);
  void setSizeRadius(float sizeRadius);
  std::vector<std::vector<float>> kalmanExtrapAccel(float time);
  std::vector<std::vector<float>> kalmanExtrapVel(float time);
  std::vector<std::vector<float>> constVelExtrap(float time);
  

  // get robot data
  cv::Point2f position();
  float angle(bool forward);
  cv::Point2f moveVel();
  cv::Point2f moveVelSlow();
  std::vector<float> getPosFiltered();
  std::vector<float> getVelFiltered();
  std::vector<float> getVelFilteredSlow();
  std::vector<float> getVelFilteredUltraSlow();
  std::vector<float> getAccFiltered();
  float turnVel();
  float turnVelSlow();
  float moveSpeed();
  float moveSpeedSlow();
  float tangentVel(bool forward);
  float tangentVelFast(bool forward);
  float moveAccel();
  float turnAccel();
  float getMaxMoveSpeed();
  float getMaxTurnSpeed();
  float getMaxMoveAccel();
  float getMaxTurnAccel();
  float getWeaponAngleReach();
  float getSizeRadius();
  std::vector<cv::Point2f> getPath();
  std::vector<float> curvatureController(float targetAngle, float moveInput, float deltaTime, bool forward, int enforceTurnDirection);
  bool pointCorrectSide(cv::Point2f point, bool CW, bool forward, float tolerance);
  void printModel();
  void setToSlowVel();
  void displayRobot(int thick, cv::Scalar sizeColor, cv::Scalar weaponColor, bool forward);





 private:

  float fieldMax = WIDTH;
  float fieldMin = 0.0f;

  float prevAngleError = 0.0f; // previous angle error for curvature controller

  // each are 3 big, XYT
  std::vector<float> posFiltered;  // current pos
  std::vector<float> velFiltered;  // current vel
  std::vector<float> accFiltered;  // current accel

  // extra filtered for threshold use
  std::vector<float> velFilteredSlow;
  std::vector<float> velFilteredUltraSlow;

  std::vector<float> modelParams; // coefficients for predictive model
  std::vector<float> modelParamScales; // estimated scales of each variable for magnitude normalizing



  // data about this robot
  float maxTurnSpeed;  // assumed turn speed when doing ETA calcs
  float maxTurnAccel;  // assumed turn accel when doing ETA calcs

  float maxMoveSpeed;  // fastest we can go
  float maxMoveAccel;  // fastest we can accel

  std::vector<cv::Point2f> path;  // tracks where robot has been
  float pathSpacing;  // how far apart are the points in the tracked path
  float pathLength;   // how long the total tracked path is

  float weaponAngleReach; // how many degrees (plus or minus) the weapon reaches from the centerline
  float sizeRadius; // radius of the frame, used for collision radius calc


  int sign(float num);
};