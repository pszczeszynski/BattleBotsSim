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
            // Blocking read until new frame available
            long frameIDnew = RobotController::GetInstance().GetIMUFrame(
                newMessage, frameID, &frameTime);

            // If a new frame was obtained do this
            if (frameIDnew >= 0)
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
    return _lastGlobalOffset;
}

void OdometryIMU::_UpdateData(IMUData &imuData, double timestamp)
{
    RobotOdometry& odometry = RobotController::GetInstance().odometry;
    OdometryData globalOdometryData = odometry.Robot(timestamp);
    CVRotation& cvRotation = odometry.GetNeuralRotOdometry();

    OdometryData cvRotData = cvRotation.GetData(false);

    cvRotData.ExtrapolateBoundedTo(timestamp);

    double neuralRotConfidence = cvRotation.GetLastConfidence();

    // Get unique access
    std::unique_lock<std::mutex> locker(_updateMutex);

    double deltaTime = timestamp - _currDataRobot.time;

    uint32_t newId = _currDataRobot.id + 1;
    _currDataRobot = OdometryData();

    // Set our rotation
    _currDataRobot._robotAngleValid = true;

    // reset the last imu angle if we changed radio channels
    if (RADIO_CHANNEL != _lastRadioChannel)
    {
        _lastImuAngle = imuData.rotation;
    }

    _lastGlobalOffset = angle_wrap(imuData.rotation - globalOdometryData._angle);

    // increment the robot angle
    _currDataRobot._angle = _lastAngle + Angle(imuData.rotation - _lastImuAngle);

    // fuse towards the neural rotation if confidence is high
    if (neuralRotConfidence > ANGLE_FUSE_CONF_THRESH)
    {
        double interpolateAmount = std::min(1.0, deltaTime * ANGLE_FUSE_SPEED);
        _currDataRobot._angle = InterpolateAngles(_currDataRobot._angle, cvRotData._angle, interpolateAmount);
    }

    _currDataRobot._robotAngleVelocity = imuData.rotationVelocity;

    _lastImuAngle = imuData.rotation;
    _lastRadioChannel = RADIO_CHANNEL;
    _lastAngle = _currDataRobot._angle;
}

void OdometryIMU::SetAngle(Angle newAngle, bool opponentRobot, double angleFrameTime, double newAngleVelocity, bool valid)
{
    // Only do this for our robot
    if (opponentRobot)
    {
        // throw exception
        throw std::runtime_error("SetAngle called for imu of opponent ??? smh");
        return;
    }

    std::unique_lock<std::mutex> locker(_updateMutex);
    _lastAngle = newAngle;
    _currDataRobot.SetAngle(newAngle, newAngleVelocity, angleFrameTime, valid);
}