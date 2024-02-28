#include "RobotOdometry.h"
#include "MathUtils.h"
#include "OpponentProfile.h"
#include "../Communication/Communication.h"
#include "RobotController.h"
#include "imgui.h"
#include "Input/InputState.h"
#include "RobotConfig.h"
#include "CVPosition.h"

RobotOdometry::RobotOdometry(ICameraReceiver &videoSource) : _videoSource(videoSource),
                                                             _odometry_Blob(&videoSource),
                                                             _odometry_Heuristic(&videoSource)
{
}

// Updates internal Odometry data
void RobotOdometry::Update(void)
{
    // ******************************
    // Retrieve new data if available

    bool newDataArrived = false;

    // Get Blob detection
    if (_odometry_Blob.IsRunning())
    {
        // Update our data
        if (_odometry_Blob.NewDataValid(_dataRobot_Blob.id, false))
        {
            _dataRobot_Blob = _odometry_Blob.GetData(false);
            newDataArrived = true;
        }

        // Update opponent data
        if (_odometry_Blob.NewDataValid(_dataOpponent_Blob.id, true))
        {
            _dataOpponent_Blob = _odometry_Blob.GetData(true);
            newDataArrived = true;
        }
    }

    // Get Heuristic detection
    if (_odometry_Heuristic.IsRunning())
    {
        // Update our data
        if (_odometry_Heuristic.NewDataValid(_dataRobot_Heuristic.id, false))
        {
            _dataRobot_Heuristic = _odometry_Heuristic.GetData(false);
            newDataArrived = true;
        }

        // Update opponent data
        if (_odometry_Heuristic.NewDataValid(_dataOpponent_Heuristic.id, true))
        {
            _dataOpponent_Heuristic = _odometry_Heuristic.GetData(true);
            newDataArrived = true;
        }
    }

    // Get IMU
    if (_odometry_IMU.IsRunning())
    {
        // Update our data
        if (_odometry_IMU.NewDataValid(_dataRobot_IMU.id, false))
        {
            _dataRobot_IMU = _odometry_IMU.GetData(false);
            newDataArrived = true;
        }
    }

    // No new data and thus nothing to do
    if (!newDataArrived)
    {
        return;
    }

    // At this time use only a priority set for all inputs
    std::unique_lock<std::mutex> locker(_updateMutex);

    if (_odometry_Heuristic.IsRunning())
    {
        _dataRobot = _dataRobot_Heuristic;
        _dataOpponent = _dataOpponent_Heuristic;
    }
    else if (_odometry_Blob.IsRunning())
    {
        _dataRobot = _dataRobot_Blob;
        _dataOpponent = _dataOpponent_Blob;
    }

    // If IMU is running, then use IMU's angle information
    if (_odometry_IMU.IsRunning() && _dataRobot_IMU.robotAngleValid)
    {
        _dataRobot.robotAngleValid = true;
        _dataRobot.robotAngle = _dataRobot_IMU.robotAngle;
        _dataRobot.robotAngleVelocity = _dataRobot_IMU.robotAngleVelocity;
    }

    // locker will get unlocked here automatically
}

/**
 * @brief returns the odometry data extrapolated to current time
 *
 */
OdometryData RobotOdometry::Robot(double currTime)
{
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData currData = _dataRobot;
    locker.unlock();

    // Extrpolate data
    currData.Extrapolate(currTime);
    return currData;
}

/**
 * @brief returns the odometry data extrapolated to current time
 *
 */
OdometryData RobotOdometry::Opponent(double currTime)
{
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData currData = _dataOpponent;
    locker.unlock();

    // Extrpolate data
    currData.Extrapolate(currTime);
    return currData;
}

static double GetImuAngleRad()
{
    // get angular velocity from imu
    return RobotController::GetInstance().GetIMUData().rotation;
}

static double GetImuAngleVelocityRadPerSec()
{
    // get angular velocity from imu
    return RobotController::GetInstance().GetIMUData().rotationVelocity;
}

Angle _AdjustAngleWithArrowKeys(Angle angle)
{
    static Clock updateClock;
    Angle angleUserAdjust = Angle(updateClock.getElapsedTime() * 30 * M_PI / 180.0);
    updateClock.markStart();
    if (InputState::GetInstance().IsKeyDown(ImGuiKey_LeftArrow))
    {
        return angle - angleUserAdjust;
    }
    else if (InputState::GetInstance().IsKeyDown(ImGuiKey_RightArrow))
    {
        return angle + angleUserAdjust;
    }

    return angle;
}

/**
 * Allows artificially setting the angle of the robot
 * This is mainly used for manual recalibration.
 *
 * @return the new angle
 */
void RobotOdometry::UpdateForceSetAngle(double newAngle, bool opponentRobot)
{
    // Go through each Odometry and update it
    _odometry_Blob.SetAngle(newAngle, opponentRobot);
    _odometry_Heuristic.SetAngle(newAngle, opponentRobot);

    // Update our own data
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData &odoData = (opponentRobot) ? _dataOpponent : _dataRobot;

    odoData.robotAngle = Angle(newAngle);
    odoData.robotAngleValid = true;
    odoData.robotAngleVelocity = 0;
}

/**
 * Allows artificially setting the position and velocity of the robot
 * This is mainly used for manual recalibration.
 *
 * @param newPos - the new position of the robot
 * @param newVel - the new velocity of the robot
 */
void RobotOdometry::UpdateForceSetPosAndVel(cv::Point2f newPos, cv::Point2f newVel, bool opponentRobot)
{
    // Go through each Odometry and update it
    _odometry_Blob.SetPosition(newPos, opponentRobot);
    _odometry_Blob.SetVelocity(newVel, opponentRobot);

    _odometry_Heuristic.SetPosition(newPos, opponentRobot);
    _odometry_Heuristic.SetVelocity(newVel, opponentRobot);

    // Update our own data
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData &odoData = (opponentRobot) ? _dataOpponent : _dataRobot;

    odoData.robotPosition = newPos;
    odoData.robotVelocity = newVel;
    odoData.robotPosValid = true;
}

// Switch position of robots
void RobotOdometry::SwitchRobots()
{
    // Switch all the Odometry
    _odometry_Blob.SwitchRobots();
    _odometry_Heuristic.SwitchRobots();

    // Update our own data
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData tempData = _dataRobot;
    _dataRobot = _dataOpponent;
    _dataOpponent = tempData;

    _dataRobot.isUs = true;
    _dataOpponent.isUs = false;
}

// Run Code
bool RobotOdometry::Run(OdometryAlg algorithm)
{
    switch (algorithm)
    {
    case OdometryAlg::Blob:
        return _odometry_Blob.Run();

    case OdometryAlg::Heuristic:
        return _odometry_Heuristic.Run();

    case OdometryAlg::IMU:
        return _odometry_IMU.Run();

    case OdometryAlg::Neural:
        break;
    }

    return false;
}

// Stop Code
bool RobotOdometry::Stop(OdometryAlg algorithm)
{
    switch (algorithm)
    {
    case OdometryAlg::Blob:
        return _odometry_Blob.Stop();

    case OdometryAlg::Heuristic:
        return _odometry_Heuristic.Stop();

    case OdometryAlg::IMU:
        return _odometry_IMU.Stop();

    case OdometryAlg::Neural:
        break;
    }

    return false;
}

// IsRunning Code
bool RobotOdometry::IsRunning(OdometryAlg algorithm)
{
    switch (algorithm)
    {
    case OdometryAlg::Blob:
        return _odometry_Blob.IsRunning();

    case OdometryAlg::Heuristic:
        return _odometry_Heuristic.IsRunning();

    case OdometryAlg::IMU:
        return _odometry_IMU.IsRunning();

    case OdometryAlg::Neural:
        break;
    }

    return false;
}

HeuristicOdometry &RobotOdometry::GetHeuristicOdometry()
{
    return _odometry_Heuristic;
}
