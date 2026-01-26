#pragma once

#include "../OdometryBase.h"

#ifdef _OPENCV_TRACKING

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/core.hpp>

#ifdef _OPENCV_TRACKING
    #if defined(CV_VERSION_MAJOR) && CV_VERSION_MAJOR >= 4
        #include <opencv2/tracking.hpp> // OpenCV 4.x
    #else
        #include <opencv2/video/tracking.hpp> // OpenCV 3.x
    #endif
#endif





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

    // Per-robot state for velocity computation (last published position/time)
    cv::Point2f _robotLastCenter{};
    double _robotLastTime = 0.0;
    bool _robotHasLast = false;

    cv::Point2f _opponentLastCenter{};
    double _opponentLastTime = 0.0;
    bool _opponentHasLast = false;

    // Epoch counters to detect TOCTOU races with SwitchRobots()/SetPosition()
    uint64_t _robotEpoch = 0;
    uint64_t _oppEpoch = 0;


};
#else // Dummy class for when OpenCV tracking is not available

class OpenCVTracker : public OdometryBase
{
public:
    OpenCVTracker(ICameraReceiver *videoSource) {};
}


#endif // _OPENCV_TRACKING
;