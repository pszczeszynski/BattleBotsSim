#include "OdometryIMU.h"
#include "../../UIWidgets/ImageWidget.h"
#include "../../RobotConfig.h"
#include "../../RobotController.h"
#include "../../RobotOdometry.h"
#include "../../CVRotation.h"

OdometryIMU::OdometryIMU() : OdometryBase(nullptr), _lastImuAngle(0)
{
}

// Start the thread
// Returns false if already running
bool OdometryIMU::Run()
{
    if (_running)
    {
        // Already running, you need to stop existing thread first
        return false;
    }

    // Start the new thread
    processingThread = std::thread([&]()
                                   {
        // Mark we are running
        _running = true;

        long frameID = -1;

        while (_running && !_stopWhenAble)
        {
            // Get the new IMU data
            double frameTime = -1.0f;

            IMUData newMessage;
            long frameIDnew = RobotController::GetInstance().GetIMUFrame(newMessage, frameID, &frameTime); // Blocking read until new frame available

            if (frameIDnew >= 0) // If a new frame was obtained do this
            {
                frameID = frameIDnew;

                // Update the data
                _UpdateData(newMessage, frameTime);
            }
        }

        // Exiting thread
        _running = false;
        _stopWhenAble = false; });

    return true;
}

/**
 * Gets the angle to add to the current angle to get the internal imu angle
*/
float OdometryIMU::GetOffset()
{
    std::unique_lock<std::mutex> locker(_updateMutex);
    return _lastImuAngle - _lastAngle;
}

void OdometryIMU::_UpdateData(IMUData &imuData, double timestamp)
{
    CVRotation& cvRotation = RobotController::GetInstance().odometry.GetNeuralRotOdometry();
    OdometryData cvRotData = cvRotation.GetData(false);
    cvRotData.Extrapolate(timestamp);
    double neuralRotConfidence = cvRotation.GetLastConfidence();

    // Get unique access
    std::unique_lock<std::mutex> locker(_updateMutex);

    double deltaTime = timestamp - _currDataRobot.time;

    _currDataRobot.id++;                // Increment frame id
    _currDataRobot.time = timestamp;    // Set to new time
    _currDataRobot.time_angle = timestamp;
    _currDataOpponent.id++;             // Increment frame id
    _currDataOpponent.time = timestamp; // Set to new time

    // Clear curr data
    _currDataRobot.Clear();
    _currDataRobot.isUs = true; // Make sure this is set
    _currDataOpponent.Clear();
    _currDataOpponent.isUs = false; // Make sure this is set

    // Set our rotation
    _currDataRobot.robotAngleValid = true;

    // reset the last imu angle if we changed radio channels
    if (RADIO_CHANNEL != _lastRadioChannel)
    {
        _lastImuAngle = imuData.rotation;
    }

    // increment the robot angle
    _currDataRobot.robotAngle = _lastAngle + Angle(imuData.rotation - _lastImuAngle);

    // fuse towards the neural rotation if confidence is high
    if (neuralRotConfidence > ANGLE_FUSE_CONF_THRESH)
    {
        double interpolateAmount = std::min(1.0, deltaTime * ANGLE_FUSE_SPEED);
        _currDataRobot.robotAngle = InterpolateAngles(_currDataRobot.robotAngle, cvRotData.robotAngle, interpolateAmount);
    }

    _currDataRobot.robotAngleVelocity = imuData.rotationVelocity;

    _lastImuAngle = imuData.rotation;
    _lastRadioChannel = RADIO_CHANNEL;
    _lastAngle = _currDataRobot.robotAngle;
}

void OdometryIMU::SetAngle(double newAngle, bool opponentRobot)
{
    // Only do this for our robot
    if (opponentRobot)
    {
        return;
    }

    std::unique_lock<std::mutex> locker(_updateMutex);
    _lastAngle = Angle(newAngle);
}