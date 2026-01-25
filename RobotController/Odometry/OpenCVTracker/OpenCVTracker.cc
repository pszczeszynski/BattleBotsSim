#include "OpenCVTracker.h"


#include "../../RobotConfig.h"

OpenCVTracker::OpenCVTracker(ICameraReceiver *videoSource) : OdometryBase(videoSource)
{
    _trackerParams.model = "dasiamrpn_model.onnx";
    _trackerParams.kernel_cls1 = "dasiamrpn_kernel_cls1.onnx";
    _trackerParams.kernel_r1 = "dasiamrpn_kernel_r1.onnx";
    _trackerParams.backend = cv::dnn::DNN_BACKEND_CUDA;
    _trackerParams.target = cv::dnn::DNN_TARGET_CUDA;

    //_robotTracker = cv::TrackerDaSiamRPN::create(_trackerParams);
    //_opponentTracker = cv::TrackerDaSiamRPN::create(_trackerParams);

    _robotTracker = cv::TrackerCSRT::create();
    _opponentTracker = cv::TrackerCSRT::create();
}

void OpenCVTracker::SwitchRobots(void)
{
    // Switch who's who
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData temp_Robot = _currDataRobot;
    _currDataRobot = _currDataOpponent;
    _currDataOpponent = temp_Robot;

    _robotTracker.swap(_opponentTracker);

    cv::Rect tempRect = _robotBBox;
    _robotBBox = _opponentBBox;
    _opponentBBox = tempRect;
}

void OpenCVTracker::SetPosition(cv::Point2f newPos, bool opponentRobot)
{
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;
    double currTime = Clock::programClock.getElapsedTime();
    
    //tracker appears to like growing to entire robot more than shrinking to only encompass it
    cv::Rect &bbox = (opponentRobot) ? _opponentBBox : _robotBBox;
    bbox.width = MIN_ROBOT_BLOB_SIZE;
    bbox.height = MIN_ROBOT_BLOB_SIZE;
    bbox.x = int(newPos.x - MIN_ROBOT_BLOB_SIZE / 2);
    bbox.y = int(newPos.y - MIN_ROBOT_BLOB_SIZE / 2);

    if (bbox.x < 0) bbox.x = 0;
    if (bbox.y < 0) bbox.y = 0;

    if (bbox.x + bbox.width > WIDTH) bbox.width = WIDTH - bbox.x;
    if (bbox.y + bbox.height > HEIGHT) bbox.height = HEIGHT - bbox.y;

    odoData.SetPosition(newPos, cv::Point2f(0, 0), bbox, currTime);
    odoData.id++;

    cv::Ptr<cv::Tracker> &tracker = (opponentRobot) ? _opponentTracker : _robotTracker;

    (opponentRobot) ? _opponentTrackerState = INIT_STARTED : _robotTrackerState = INIT_STARTED;
}

void OpenCVTracker::_ProcessNewFrame(cv::Mat currFrame, double frameTime)
{
    if (currFrame.channels() == 1)
    {
        cv::cvtColor(currFrame, _previousImage, cv::COLOR_GRAY2RGB);
    }
    else if (currFrame.channels() == 3)
    {
        _previousImage = currFrame;
    }
    else if (currFrame.channels() == 4)
    {
        cv::cvtColor(currFrame, _previousImage, cv::COLOR_RGBA2RGB);
    }

    if (_robotTrackerState == INIT_STARTED)
    {
        _robotTracker->init(_previousImage, _robotBBox);
        _robotTrackerState = INITIALIZED;
    }
    else if (_robotTrackerState == INITIALIZED)
    {
        bool valid = _robotTracker->update(_previousImage, _robotBBox);
        cv::Point2f center(_robotBBox.x + _robotBBox.width / 2.0f, 
                          _robotBBox.y + _robotBBox.height / 2.0f);
        if (valid) {
            _currDataRobot.SetPosition(center, cv::Point2f(0, 0), _robotBBox, frameTime);
        } else {
            _currDataRobot.InvalidatePosition();
        }
        _currDataRobot.id++;
    }

    if (_opponentTrackerState == INIT_STARTED)
    {
       _opponentTracker->init(_previousImage, _opponentBBox);
       _opponentTrackerState = INITIALIZED;
    }
    else if (_opponentTrackerState == INITIALIZED)
    {
        bool valid = _opponentTracker->update(_previousImage, _opponentBBox);
        cv::Point2f center(_opponentBBox.x + _opponentBBox.width / 2.0f, 
                          _opponentBBox.y + _opponentBBox.height / 2.0f);
        if (valid) {
            _currDataOpponent.SetPosition(center, cv::Point2f(0, 0), _opponentBBox, frameTime);
        } else {
            _currDataOpponent.InvalidatePosition();
        }
        _currDataOpponent.id++;
    }   
}