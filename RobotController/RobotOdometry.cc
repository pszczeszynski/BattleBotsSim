#include "RobotOdometry.h"
#include "MathUtils.h"
#include "OpponentProfile.h"
#include "../Communication/Communication.h"
#include "RobotController.h"
#include "imgui.h"
#include "Input/InputState.h"
#include "RobotConfig.h"

RobotOdometry::RobotOdometry(ICameraReceiver& videoSource) :
    _videoSource(videoSource), 
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
    if( _odometry_Blob.IsRunning() )
    {
        // Update our data
        if( _odometry_Blob.NewDataValid(_dataRobot_Blob.id, false))
        {
            _dataRobot_Blob =  _odometry_Blob.GetData(false);
            newDataArrived = true;
        }

        // Update opponent data
        if( _odometry_Blob.NewDataValid(_dataOpponent_Blob.id, true))
        {
            _dataOpponent_Blob =  _odometry_Blob.GetData(true);
            newDataArrived = true;
        }
    }

    // Get Heuristic detection
    if( _odometry_Heuristic.IsRunning() )
    {
        // Update our data
        if( _odometry_Heuristic.NewDataValid(_dataRobot_Heuristic.id, false))
        {
            _dataRobot_Heuristic =  _odometry_Heuristic.GetData(false);
            newDataArrived = true;
        }

        // Update opponent data
        if( _odometry_Heuristic.NewDataValid(_dataOpponent_Heuristic.id, true))
        {
            _dataOpponent_Heuristic =  _odometry_Heuristic.GetData(true);
            newDataArrived = true;
        }
    }

    // No new data and thus nothing to do
    if(!newDataArrived) { return;}

    // At this time use only a priority set for all inputs
    std::unique_lock<std::mutex> locker(_updateMutex);

    if( _odometry_Heuristic.IsRunning())
    {
        _dataRobot =_dataRobot_Heuristic;
        _dataOpponent = _dataOpponent_Heuristic;
    }
    else if(_odometry_Blob.IsRunning() )
    {
        _dataRobot =_dataRobot_Blob;
        _dataOpponent = _dataOpponent_Blob;
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
    currData.Extrapolate( currTime);
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
    currData.Extrapolate( currTime);
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
 * @brief updates with both visual and imu information. this should only be called for our robot (since we have our imu)
 * However, sometimes we don't have the visual information in which we call UpdateIMUOnly
*/
#define FUSE_ANGLE_WEIGHT 0.15

void RobotOdometry::UpdateVisionAndIMU(MotionBlob& blob, cv::Mat& frame)
{
/*
    //////////////////////// POS ////////////////////////
    cv::Point2f visualPos = blob.center;
    cv::Point2f predictedPosition = _position + _lastVelocity * _lastUpdateClock.getElapsedTime();

    bool valid = _IsValidBlob(blob);
    valid = true;

    //////////////////////// VEL ////////////////////////
    cv::Point2f smoothedVisualVelocity = _GetSmoothedVisualVelocity(blob);
    if (!valid)
    {
        smoothedVisualVelocity = _lastVelocity;
        visualPos = _position;
    }

    /////////////////////// ANGLE ///////////////////////
    // set the fused angle to the imu angle
    double fusedAngle = _UpdateAndGetIMUAngle();
    // calculate angle using the visual information
    double visualAngle = CalcAnglePathTangent();
    // if can use the visual information and the user presses enter (to realign)
    if (_visualAngleValid && (ImGui::IsKeyDown(ImGuiKey::ImGuiKey_LeftCtrl) || RobotController::GetInstance().gamepad.GetDpadLeft()))
    {
        // if we have visual information, use the visual angle
        fusedAngle = InterpolateAngles(Angle(fusedAngle), Angle(visualAngle), FUSE_ANGLE_WEIGHT);
    }

    // if we should use the rotation network
    if (ROTATION_NET_ENABLED)
    {
        std::cout << "Computing robot rotation" << std::endl;
        // use the ml model to get the angle entirely
        fusedAngle = CVRotation::GetInstance().ComputeRobotRotation(RobotController::GetInstance().GetDrawingImage(), visualPos);
    }

    // update using the weighted average
    _PostUpdate(visualPos, smoothedVisualVelocity, Angle(fusedAngle));
    */
}

/**
 * Called when we only have the imu information. This should only be called for our robot (since we have our imu)
*/
void RobotOdometry::UpdateIMUOnly(cv::Mat& frame)
{
/*    //////////////////////// VEL ////////////////////////
    // just use the last velocity
    cv::Point2f velocity = _lastVelocity;

    //////////////////////// ANGLE ////////////////////////
    // set the angle using just the imu
    Angle angle = Angle(_UpdateAndGetIMUAngle());

    // angle = Angle(CVRotation::GetInstance().ComputeRobotRotation(RobotController::GetInstance().GetDrawingImage(), _position));

    // update normally
    _PostUpdate(_position, velocity, angle);
*/
}

/**
 * Tracks the change in the imu angle
 * Should be called every time we update using an imu
 * 
 * @return the new angle with the change in angle added
*/
double RobotOdometry::_UpdateAndGetIMUAngle()
{
/*
    // 1. compute angle change
    // get the new angle
    double newAngleImuRad = GetImuAngleRad();
    // calculate the change in angle (there are no angle wraps since the imu is continuous)
    double angleChange = newAngleImuRad - _lastIMUAngle;
    // update the last angle
    _lastIMUAngle = newAngleImuRad;

    // 2. compute and save angular velocity for later
    _angleVelocity = GetImuAngleVelocityRadPerSec();

    // 3. return the current angle + change in angle
    return _angle + angleChange;
*/
    return 0;
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
    OdometryData& odoData = (opponentRobot) ? _dataOpponent : _dataRobot;

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
    OdometryData& odoData = (opponentRobot) ? _dataOpponent : _dataRobot;

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
    _dataRobot =_dataOpponent;
    _dataOpponent = tempData;

    _dataRobot.isUs = true;
    _dataOpponent.isUs = false;
}

// Run Code
bool RobotOdometry::Run(OdometryAlg algorithm)
{
    switch( algorithm ){
        case OdometryAlg::Blob:
            return _odometry_Blob.Run();
            
        case OdometryAlg::Heuristic:
            return _odometry_Heuristic.Run();            

        case OdometryAlg::Neural:
            break;

    }

    return false;
}

// Stop Code
bool RobotOdometry::Stop(OdometryAlg algorithm)
{
    switch( algorithm ){
        case OdometryAlg::Blob:
            return _odometry_Blob.Stop();
            
        case OdometryAlg::Heuristic:
            return _odometry_Heuristic.Stop();

        case OdometryAlg::Neural:
            break;

    }

    return false;
}

// IsRunning Code
bool RobotOdometry::IsRunning(OdometryAlg algorithm)
{
    switch( algorithm ){
        case OdometryAlg::Blob:
            return _odometry_Blob.IsRunning();
            
        case OdometryAlg::Heuristic:
            return _odometry_Heuristic.IsRunning();

        case OdometryAlg::Neural:
            break;

    }

    return false;
}

HeuristicOdometry& RobotOdometry::GetHeuristicOdometry()
{
    return _odometry_Heuristic;
}

