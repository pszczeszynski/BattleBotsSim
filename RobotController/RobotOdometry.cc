#include "RobotOdometry.h"
#include "MathUtils.h"
#include "OpponentProfile.h"
#include "../Common/Communication.h"
#include "RobotController.h"
#include "imgui.h"
#include "Input/InputState.h"
#include "RobotConfig.h"
#include "odometry/Neural/CVPosition.h"
#include "UIWidgets/GraphWidget.h"
#include "UIWidgets/ImageWidget.h"
#include "Globals.h"

RobotOdometry::RobotOdometry(ICameraReceiver &videoSource) : _videoSource(videoSource),
                                                             _odometry_Blob(&videoSource),
                                                             _odometry_Heuristic(&videoSource),
                                                             _odometry_Neural(&videoSource),
                                                             _odometry_Human(&videoSource)
{
}

void RobotOdometry::_AdjustAngleWithArrowKeys()
{
    static Clock updateClock;
    Angle angleUserAdjust = Angle(updateClock.getElapsedTime() * 90 * M_PI / 180.0);
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
    FuseAndUpdatePositions();

    // If IMU is running, then use IMU's angle information
    if (_odometry_IMU.IsRunning() && _dataRobot_IMU.robotAngleValid)
    {
        _dataRobot.robotAngleValid = true;
        _dataRobot.robotAngle = _dataRobot_IMU.robotAngle;
        _dataRobot.robotAngleVelocity = _dataRobot_IMU.robotAngleVelocity;
    }



    // locker will get unlocked here automatically
}

void RobotOdometry::FuseAndUpdatePositions()
{
    OdometryData* candidateRobot = &_dataRobot_Heuristic;

    bool heuristicValid = _odometry_Heuristic.IsRunning() && _dataRobot_Heuristic.robotPosValid;
    bool blobValid = _odometry_Blob.IsRunning(); // don't include valid, since it might just be stopped

    // check if the neural net is far away from the candidate, override to neural net if yes
    double currTimeSeconds = Clock::programClock.getElapsedTime();
    double ageOfNeural = currTimeSeconds - _dataRobot_Neural.time;
    // the neural is valid if it isn't too old && it's running
    bool neuralValid = _odometry_Neural.IsRunning() && _dataRobot_Neural.robotPosValid && ageOfNeural < 0.3;

    // IF only heuristic is valid, the candidate is the heuristic
    if (heuristicValid && !blobValid)
    {
        candidateRobot = &_dataRobot_Heuristic;
    }
    // IF only blob is valid, the candidate is blob
    else if (blobValid && !heuristicValid)
    {
        candidateRobot = &_dataRobot_Blob;
    }
    // IF both are valid
    else if (heuristicValid && blobValid)
    {
        // if neural net is running and valid, take the one closer to the neural net
        if (neuralValid)
        {
            OdometryData robotNeural = _dataRobot_Neural;
            // extrapolate to current time
            robotNeural.Extrapolate(Clock::programClock.getElapsedTime());
            cv::Point2f robotPosNeural = robotNeural.robotPosition;

            double distHeuristic = cv::norm(_dataRobot_Heuristic.robotPosition - robotPosNeural);
            double distBlob = cv::norm(_dataRobot_Blob.robotPosition - robotPosNeural);

            // choose the one closer to the neural net
            if (distHeuristic < distBlob)
            {
                candidateRobot = &_dataRobot_Heuristic;
            }
            else
            {
                candidateRobot = &_dataRobot_Blob;
            }
        }
    }

    // if the neural net is valid, then check if it is far away from the candidate
    if (neuralValid)
    {
        OdometryData robotNeural = _dataRobot_Neural;
        // extrapolate to current time
        robotNeural.Extrapolate(Clock::programClock.getElapsedTime());
        cv::Point2f robotPosNeural = robotNeural.robotPosition;

        // compute distance from the current candidate
        double distCandidate = cv::norm(candidateRobot->robotPosition - robotPosNeural);

        // if far away, then use the neural net
        if (distCandidate > 50)
        {
            candidateRobot = &_dataRobot_Neural;

            // call set position on the other two algorithms so we re-lock
            _odometry_Blob.SetPosition(candidateRobot->robotPosition, false);
            _odometry_Heuristic.SetPosition(candidateRobot->robotPosition, false);
        }
    }


    cv::Scalar color = cv::Scalar(0, 255, 0);
    std::string algorithmName = "None";
    if (candidateRobot == &_dataRobot_Blob)
    {
        // set to blue
        color = cv::Scalar(0, 0, 255);
        algorithmName = "Blob";
    }
    else if (candidateRobot == &_dataRobot_Heuristic)
    {
        // set to yellow - orange
        color = cv::Scalar(0, 180, 255);
        algorithmName = "Heuristic";
    }
    else if (candidateRobot == &_dataRobot_Neural)
    {
        // set to purple
        color = cv::Scalar(255, 0, 255);
        algorithmName = "Neural";
    }

    TrackingWidget* trackingWidget = TrackingWidget::GetInstance();

    cv::Mat trackingMat;
    if (trackingWidget)
    {
        trackingMat = TrackingWidget::GetInstance()->GetTrackingMat();

        // put text at top center of the image
        cv::putText(trackingMat, "Robot using " + algorithmName, cv::Point(trackingMat.cols / 2 - 75, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

        // draw robot angle with an arrow
        cv::Point2f arrowEnd = _dataRobot.robotPosition + cv::Point2f(50 * cos(_dataRobot_IMU.robotAngle), 50 * sin(_dataRobot_IMU.robotAngle));
        cv::arrowedLine(trackingMat, _dataRobot.robotPosition, arrowEnd, color, 2);
    }

    _dataRobot = *candidateRobot;





    //////////////// OPPONENT //////////////////
    OdometryData* candidateOpponent = &_dataOpponent_Blob;

    heuristicValid = _odometry_Heuristic.IsRunning() && _dataOpponent_Heuristic.robotPosValid;
    blobValid = _odometry_Blob.IsRunning(); // don't include valid, since it might just be stopped

    // IF only heuristic is valid, the candidate is the heuristic
    if (heuristicValid && !blobValid)
    {
        candidateOpponent = &_dataOpponent_Heuristic;
    }
    // IF only blob is valid, the candidate is blob
    else if (blobValid && !heuristicValid)
    {
        candidateOpponent = &_dataOpponent_Blob;
    }
    // IF both are valid
    else if (heuristicValid && blobValid)
    {
        OdometryData oldOpponent = _dataOpponent;
        // extrapolate the old opponent to current time
        oldOpponent.Extrapolate(Clock::programClock.getElapsedTime());

        // choose the one closer to the old opponent
        double distHeuristic = cv::norm(_dataOpponent_Heuristic.robotPosition - oldOpponent.robotPosition);
        double distBlob = cv::norm(_dataOpponent_Blob.robotPosition - oldOpponent.robotPosition);

        if (distHeuristic < distBlob)
        {
            candidateOpponent = &_dataOpponent_Heuristic;
        }
        else
        {
            candidateOpponent = &_dataOpponent_Blob;
        }
    }


    std::string algorithmNameOpponent = "None";
    cv::Scalar colorOpponent = cv::Scalar(0, 255, 0);

    if (candidateOpponent == &_dataOpponent_Blob)
    {
        // set to blue
        colorOpponent = cv::Scalar(0, 0, 255);
        algorithmNameOpponent = "Blob";
    }
    else if (candidateOpponent == &_dataOpponent_Heuristic)
    {
        // set to yellow - orange
        colorOpponent = cv::Scalar(0, 180, 255);
        algorithmNameOpponent = "Heuristic";
    }



    _dataOpponent = *candidateOpponent;   

    if (!trackingMat.empty())
    {
        // put text at top center of the image
        cv::putText(trackingMat, "Opponent using " + algorithmNameOpponent, cv::Point(trackingMat.cols / 2 - 75, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        int size = (MIN_ROBOT_BLOB_SIZE + MAX_ROBOT_BLOB_SIZE) / 4;
        // draw on the tracking mat a square
        cv::rectangle(trackingMat, _dataRobot.robotPosition - cv::Point2f(size, size), _dataRobot.robotPosition + cv::Point2f(size, size), cv::Scalar(255, 255, 255), 2);


        size = (MIN_OPPONENT_BLOB_SIZE + MAX_OPPONENT_BLOB_SIZE) / 4;

        // draw a circle for the opponent
        cv::circle(trackingMat, _dataOpponent.robotPosition, size, cv::Scalar(255, 255, 255), 2);
    }
}

/*
 * Returns true if the tracking data can be trusted + used for orbit + kill mode
 * False otherwise.
 * 
 * Criteria are:
 * 1. Good if both blob + heuristic are valid + agree, and neural net isn't running
 * 2. Good if the current selected robot pos with the neural net + neural net valid + recent
 * 3. Bad otherwise
 */
bool RobotOdometry::IsTrackingGoodQuality()
{
    const double AGREEMENT_DIST_THRESH_PX = 50;

    bool heuristicValid = _odometry_Heuristic.IsRunning() && _dataRobot_Heuristic.robotPosValid && _dataRobot_Heuristic.GetAge() < 0.3;
    bool blobValid = _odometry_Blob.IsRunning(); // don't include valid, since it might just be stopped
    // the neural is valid if it isn't too old && it's running
    bool neuralValid = _odometry_Neural.IsRunning() && _dataRobot_Neural.robotPosValid && _dataRobot_Neural.GetAge() < 0.3;

    // std::cout << "heuristicValid: " << heuristicValid << std::endl;
    // std::cout << "blobValid: " << blobValid << std::endl;
    // std::cout << "neuralValid: " << neuralValid << std::endl;
    // std::cout << "heuristic robot pos valid? " << _dataRobot_Heuristic.robotPosValid << std::endl;
    // std::cout << "heuristic age: " << _dataRobot_Heuristic.GetAge() << std::endl;
    // if everybody is crying, return false :(
    if (!heuristicValid && !blobValid && !neuralValid)
    {
        // std::cout << "case 1" << std::endl;
        return false;
    }

    // if heuristic and blob valid, but neural net isn't running
    if (heuristicValid && blobValid && !neuralValid)
    {
        double distBetweenRobot = cv::norm(_dataRobot_Heuristic.robotPosition - _dataRobot_Blob.robotPosition);
        double distBetweenOpponent = cv::norm(_dataOpponent_Heuristic.robotPosition - _dataOpponent_Blob.robotPosition);

        // std::cout << "distBetweenRobot: " << distBetweenRobot << std::endl;
        // std::cout << "distBetweenOpponent: " << distBetweenOpponent << std::endl;
        // return true if they agree, false otherwise
        return distBetweenRobot < AGREEMENT_DIST_THRESH_PX && distBetweenOpponent < AGREEMENT_DIST_THRESH_PX;
    }

    // if the neural net is running and predicting
    if (neuralValid)
    {
        // check for agreement with neural net + agreement between the other two algorithms for the opponent
        double distToRobot = cv::norm(_dataRobot.robotPosition - _dataRobot_Neural.robotPosition);
        double distBetweenOpponent = cv::norm(_dataOpponent_Heuristic.robotPosition - _dataOpponent_Blob.robotPosition);
        // std::cout << "distToRobot: " << distToRobot << std::endl;
        // std::cout << "distBetweenOpponent: " << distBetweenOpponent << std::endl;

        return distToRobot < AGREEMENT_DIST_THRESH_PX && distBetweenOpponent < AGREEMENT_DIST_THRESH_PX;
    }

    // std::cout << "default case" << std::endl;
    // otherwise return false, since we don't have enough confidence
    return false;
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

    case OdometryAlg::Human:
        return _odometry_Human.Run();
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
        return _odometry_Neural.Stop();
    
    case OdometryAlg::Human:
        return _odometry_Human.Stop();
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
        return _odometry_Neural.IsRunning();
    
    case OdometryAlg::Human:
        return _odometry_Human.IsRunning();
    
    default:
        break;
    }

    return false;
}

HeuristicOdometry &RobotOdometry::GetHeuristicOdometry()
{
    return _odometry_Heuristic;
}

CVPosition &RobotOdometry::GetNeuralOdometry()
{
    return _odometry_Neural;
}

BlobDetection &RobotOdometry::GetBlobOdometry()
{
    return _odometry_Blob;
}



/**
 * @brief Forces the position of a tracking algorithm to be a certain value
 * 
 * @param alg - the algorithm to set the position of
 * @param pos - the position to set it to
 * @param opponent - whether to set the opponent or not
 */
void RobotOdometry::ForceSetPositionOfAlg(OdometryAlg alg, cv::Point2f pos, bool opponent)
{
    if (alg == OdometryAlg::Blob)
    {
        _odometry_Blob.SetPosition(pos, opponent);
    }
    else if (alg == OdometryAlg::Heuristic)
    {
        _odometry_Heuristic.SetPosition(pos, opponent);
    }
    else if (alg == OdometryAlg::IMU)
    {
        _odometry_IMU.SetPosition(pos, opponent);
    }
    else if (alg == OdometryAlg::Neural)
    {
        _odometry_Neural.SetPosition(pos, opponent);
    }
}


/**
 * @brief Forces the velocity of a tracking algorithm to be a certain value
 * 
 * @param alg - the algorithm to set the velocity of
 * @param vel - the velocity to set it to
 * @param opponent - whether to set the opponent or not
 */
void RobotOdometry::ForceSetVelocityOfAlg(OdometryAlg alg, cv::Point2f vel, bool opponent)
{
    if (alg == OdometryAlg::Blob)
    {
        _odometry_Blob.SetVelocity(vel, opponent);
    }
    else if (alg == OdometryAlg::Heuristic)
    {
        _odometry_Heuristic.SetVelocity(vel, opponent);
    }
    else if (alg == OdometryAlg::IMU)
    {
        _odometry_IMU.SetVelocity(vel, opponent);
    }
    else if (alg == OdometryAlg::Neural)
    {
        _odometry_Neural.SetVelocity(vel, opponent);
    }
}
