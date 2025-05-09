#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "../../Globals.h"
#include "../OdometryBase.h"
#include "RobotClassifier.h"
#include "../../CameraReceiver.h"
#include "../../VisionClassification.h"
#include "MotionBlob.h"

// BlobDetection Odometry
// Tracks objects based on the blob movement
class BlobDetection : public OdometryBase
{
public:
    BlobDetection(ICameraReceiver *videoSource);

    void SwitchRobots(void) override;
    void SetPosition(cv::Point2f newPos, bool opponentRobot) override;
    void SetVelocity(cv::Point2f newVel, bool opponentRobot) override;
    void SetAngle(double newAngle, bool opponentRobot) override;

private:
    void _ProcessNewFrame(cv::Mat currFrame, double frameTime) override; // Run every time a new frame is available

    VisionClassification DoBlobDetection(cv::Mat &currFrame, cv::Mat &prevFrame, std::unique_lock<std::mutex> &locker, double frameTime); // The core blob detection algorithm
    void UpdateData(VisionClassification robotData, double timestamp);            // Updates the core data so others can poll it

    bool _IsValidBlob(MotionBlob &blobNew, OdometryData &prevData);
    // bool _IsValidBlob(MotionBlob &blobNew, OdometryData &currData, OdometryData &prevData); // Checks if blob is valid
    void _GetSmoothedVisualVelocity(OdometryData &currData, OdometryData &prevData);        // Averages velocity since its comming in jittery
    void SetData(MotionBlob *blob, OdometryData &currData, OdometryData &prevData);
    void CalcAnglePathTangent(OdometryData &currData, OdometryData &prevData);

    RobotClassifier _robotClassifier; // Takes the blobs and figures out whos who
    cv::Mat _previousImage;           // The previous image to do delta on
    Clock _prevImageTimer;            // Keeps track of when we updated our previous image
    double prevFrameTime = 0;         // The previous time for velocity calcs

    cv::Mat _lastDrawingImage;      // Copy of the last image for other processes to pull
    OdometryData _prevDataRobot;    // Contains the previous data, useful for velocity calculations, etc..
    OdometryData _prevDataOpponent; // Contains the previous data, useful for velocity calculations, etc..

    OdometryData _tempData;
};
