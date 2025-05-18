#pragma once

#include "../../Globals.h"
#include "../OdometryBase.h"

#ifdef _OPENCV_TRACKING

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>



enum TrackerState
{
    UNINITIALIZED = 0,
    INIT_STARTED,
    INITIALIZED
};

class OpenCVTracker : public OdometryBase
{
public:
    OpenCVTracker(ICameraReceiver *videoSource);

    void SwitchRobots(void) override;
    void SetPosition(cv::Point2f newPos, bool opponentRobot) override;

private:
    void _ProcessNewFrame(cv::Mat currFrame, double frameTime) override;

    cv::Ptr<cv::Tracker> _robotTracker;
    cv::Ptr<cv::Tracker> _opponentTracker;
    cv::TrackerDaSiamRPN::Params _trackerParams;

    cv::Rect _robotBBox;
    cv::Rect _opponentBBox;

    cv::Mat _previousImage;

    // Tracker needs initial bbox to start
    // indicates whether the bbox has been given since launch
    enum TrackerState _robotTrackerState = TrackerState::UNINITIALIZED;
    enum TrackerState _opponentTrackerState = TrackerState::UNINITIALIZED;


};
#else // Dummy class for when OpenCV tracking is not available

class OpenCVTracker : public OdometryBase
{
public:
    OpenCVTracker(ICameraReceiver *videoSource) {};
}


#endif // _OPENCV_TRACKING
;