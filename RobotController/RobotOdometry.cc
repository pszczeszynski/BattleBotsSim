#include "RobotOdometry.h"
#include "MathUtils.h"
#include "OpponentProfile.h"
#include "../Communication/Communication.h"
#include "RobotController.h"
#include "imgui.h"
#include "Input/InputState.h"
#include "RobotConfig.h"
#include "odometry/Neural/CVPosition.h"
#include "UIWidgets/GraphWidget.h"

RobotOdometry::RobotOdometry(ICameraReceiver &videoSource) : _videoSource(videoSource),
                                                             _odometry_Blob(&videoSource),
                                                             _odometry_Heuristic(&videoSource)
{
}

void RobotOdometry::_AdjustAngleWithArrowKeys()
{
    static Clock updateClock;
    Angle angleUserAdjust = Angle(updateClock.getElapsedTime() * 30 * M_PI / 180.0);
    updateClock.markStart();
    if (InputState::GetInstance().IsKeyDown(ImGuiKey_LeftArrow))
    {
        UpdateForceSetAngle(_dataRobot.robotAngle - angleUserAdjust, false);
    }
    else if (InputState::GetInstance().IsKeyDown(ImGuiKey_RightArrow))
    {
        UpdateForceSetAngle(_dataRobot.robotAngle + angleUserAdjust, false);
    }
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

    // Get Neural
    if (_odometry_Neural.IsRunning())
    {
        // Update our data
        if (_odometry_Neural.NewDataValid(_dataRobot_Neural.id, false))
        {
            _dataRobot_Neural = _odometry_Neural.GetData(false);
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

    // if neural data is available and far away from the robot
    if (_odometry_Neural.IsRunning() && _dataRobot_Neural.robotPosValid)
    {
        _dataRobot = _dataRobot_Neural;
        // // get the distance from the robot to the neural position
        // float distanceToRobot = cv::norm(_dataRobot_Neural.robotPosition - _dataRobot.robotPosition);
        // cv::circle(RobotController::GetInstance().GetDrawingImage(), _dataRobot_Neural.robotPosition, 20, cv::Scalar(255, 0, 0), 2);
        // // if the distance is greater than 50 pixels
        // if (distanceToRobot > 70)
        // {
        //     float distanceToOpponent = cv::norm(_dataRobot_Neural.robotPosition - _dataOpponent.robotPosition);

        //     // if (distanceToRobot > distanceToOpponent)
        //     // {
        //     //     _odometry_Blob.SwitchRobots();
        //     //     _odometry_Blob.SetPosition(neuralPos, false);
        //     // }

        //     if (_odometry_Blob.IsRunning())
        //     {
        //         _odometry_Blob.SetPosition(_dataRobot_Neural.robotPosition, false);
        //     }
        //     else if (_odometry_Heuristic.IsRunning())
        //     {
        //         _odometry_Heuristic.SetPosition(_dataRobot_Neural.robotPosition, false);
        //     }
        // }
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
 */
#define MAX_EXTRAPOLATION_TIME_S 0.1
OdometryData RobotOdometry::Robot(double currTime)
{
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData currData = _dataRobot;
    locker.unlock();

    double extrapolateTime = min(currTime - currData.time, MAX_EXTRAPOLATION_TIME_S);
    currData.Extrapolate(extrapolateTime);

    return currData;
}

/**
 * @brief Returns the angle to add to the global angle to get the internal imu angle (in radians)
*/
float RobotOdometry::GetIMUOffset()
{
    return _odometry_IMU.GetOffset();
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

    double extrapolateTime = min(currTime - currData.time, MAX_EXTRAPOLATION_TIME_S);
    currData.Extrapolate(extrapolateTime);

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
    _odometry_IMU.SetAngle(newAngle, opponentRobot);

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
        return _odometry_Neural.Run();
    default:
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
