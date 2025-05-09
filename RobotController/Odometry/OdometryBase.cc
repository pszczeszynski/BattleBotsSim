#include "OdometryBase.h"
#include <opencv2/core.hpp>

// clock widget
#include "../UIWidgets/ClockWidget.h"

// ctor for odometry data
OdometryData::OdometryData() : robotPosition(-1.0f, -1.0f), robotVelocity(0, 0)
{
}

// Clear all position data
void OdometryData::Clear()
{
    // Our Position
    robotPosValid = false;
    robotPosition = cv::Point2f(-1.0f, -1.0f);
    robotVelocity = cv::Point2f(0, 0); // pixels/second

    rect = cv::Rect2i(0, 0, 0, 0);

    // Our Rotation
    robotAngleValid = false;
    robotAngle = Angle(0);
    robotAngleVelocity = 0; // Clockwise, rads/s

    // Clear user data
    userDataDouble.clear();
}

// Modifies internal data to extrapolate values
void OdometryData::Extrapolate(double newtime)
{
    // Only extrapolate into the future
    if (newtime < time)
    {
        newtime = time;
    }

    // Extrapolate only if data is marked as valid
    if (robotPosValid)
    {
        robotPosition += robotVelocity * (newtime - time);
    }

    time = newtime;

    // Now do angle
    double angleTime = (time_angle < 0) ? time : time_angle;
    if (newtime < angleTime)
    {
        newtime = angleTime;
    }

    if (robotAngleValid)
    {
        robotAngle = robotAngle + Angle(robotAngleVelocity * (newtime - time_angle));
    }

    time_angle = newtime;
}

void OdometryData::ExtrapolateBounded(double newtime, double maxRelativeTime)
{
    if (newtime > time + maxRelativeTime)
    {
        return;
    }

    Extrapolate(newtime);
}


/**
 * Returns the age of this data in seconds
*/
double OdometryData::GetAge()
{
    return ClockWidget::programClock.getElapsedTime() - time;
}

// ***********************************************
// ************ Odometry Base ********************
// ***********************************************

OdometryBase::OdometryBase(ICameraReceiver *videoSource) : _videoSource(videoSource)
{
    _currDataRobot.isUs = true;     // Make sure this is set
    _currDataOpponent.isUs = false; // Make sure this is set
};

bool OdometryBase::IsRunning(void)
{
    return _running;
}

// Start the thread
// Returns false if already running
bool OdometryBase::Run(void)
{
    if (_running)
    {
        // Already running, you need to stop existing thread first
        return false;
    }

    if( _videoSource == nullptr )
    {
        // No video source, cannot run
        return false;
    }

    // Start the new thread
    processingThread = std::thread([&]()
                                   {
        // Mark we are running
        _running = true;

        // Initialize anything that needs initilialization
        _StartCalled();

        frameID = -1;

        while (_running && !_stopWhenAble)
        {
            // Get the new frame video frame
            // Iats going to be pre-processed already (e.g. birdseyeview) and black-and-white
            cv::Mat currFrame;
            double frameTime = -1.0f;
            frameID = _videoSource->GetFrame(currFrame, frameID, &frameTime, false); // Blocking read with timeout

            // Process the new frame
            if ((frameID > -1) && !currFrame.empty())
            {
                _ProcessNewFrame(currFrame, frameTime);
            }
        }

        // Call any stop routines
        _StopCalled();

        // Exiting thread
        _running = false;
        _stopWhenAble = false; });

    return true;
}

// Stop the main thread
bool OdometryBase::Stop(void)
{
    // If not running, return true
    if (!_running)
    {
        return true;
    }

    // Try to stop the thread
    _stopWhenAble = true;

    Clock clockWaiting;

    // Wait with a timeout to have it end
    while (_stopWhenAble && (clockWaiting.getElapsedTime() < ODOMETRY_STOP_TIMEOUT))
    {
        // Sleep for 1ms
        Sleep(1);
    }

    // See if it ended. The thread may have died thus also check if its joinable.
    if (!_stopWhenAble || processingThread.joinable())
    {
        processingThread.join();

        // Reset these in case the thread died and didnt reset it
        _running = false;
        _stopWhenAble = false;

        return true;
    }

    // It didnt end!
    return false;
}

// Check if new data is available
bool OdometryBase::NewDataValid(int oldId, bool getOpponent)
{
    // Mutex not required since we're just comparing an int
    if (getOpponent)
    {
        return _currDataOpponent.id > oldId;
    }

    return _currDataRobot.id > oldId;
}

//  Retrieve the actual data
OdometryData OdometryBase::GetData(bool getOpponent)
{
    // Get acccess to private data
    std::unique_lock<std::mutex> locker(_updateMutex);

    // Return it (via copy operator)
    return (getOpponent) ? _currDataOpponent : _currDataRobot;
}

void OdometryBase::SwitchRobots(void)
{
    // Switch who's who
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData temp_Robot = _currDataRobot;
    _currDataRobot = _currDataOpponent;
    _currDataOpponent = temp_Robot;

    _currDataRobot.isUs = true;
    _currDataOpponent.isUs = false;
}

// Set postion recommends newPos to be the center of the robot. The algorithm is free to adjust
// as required (e.g. find closest tracking rectangle and use its center)
void OdometryBase::SetPosition(cv::Point2f newPos, bool opponentRobot)
{
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;

    odoData.robotPosition = newPos;
    odoData.robotPosValid = true;
}

// Force position forces the center of robot to be newpos regardlesss of any other considerations
void OdometryBase::ForcePosition(cv::Point2f newPos, bool opponentRobot)
{
    SetPosition(newPos, opponentRobot);
}


void OdometryBase::SetVelocity(cv::Point2f newVel, bool opponentRobot)
{
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;

    odoData.robotVelocity = newVel;
}

// Sets angle and zeroes out angular velocity
void OdometryBase::SetAngle(double newAngle, bool opponentRobot)
{
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;

    odoData.robotAngle = Angle(newAngle);
    odoData.robotAngleValid = true;
    odoData.robotAngleVelocity = 0;
}


void OdometryBase::SetAngularVelocity(double newVel, bool opponentRobot)
{
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;

    odoData.robotAngleVelocity = newVel;
}
