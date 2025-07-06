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

    _currDataRobot = OdometryData(_currDataRobot.id + 1);
    // reset the last imu angle if we changed radio channels
    if (RADIO_CHANNEL != _lastRadioChannel)
    {
        _lastImuAngle = imuData.rotation;
    }

    _lastGlobalOffset = angle_wrap(imuData.rotation - globalOdometryData.GetAngle());

    // increment the robot angle (corrects for any fixed dc offset)
    Angle newAngle = _lastAngle + Angle(imuData.rotation - _lastImuAngle);

    // fuse towards the neural rotation if confidence is high
    if (neuralRotConfidence > ANGLE_FUSE_CONF_THRESH)
    {
        double interpolateAmount = std::min(1.0, deltaTime * ANGLE_FUSE_SPEED);
        newAngle = InterpolateAngles(newAngle, cvRotData.GetAngle(), interpolateAmount);
    }

    // set the new angle + velocity
    _currDataRobot.SetAngle(newAngle, imuData.rotationVelocity, timestamp, true);
    _currDataRobot.time = timestamp;

    _lastImuAngle = imuData.rotation;
    _lastRadioChannel = RADIO_CHANNEL;
    _lastAngle = _currDataRobot.GetAngle();
}

void OdometryIMU::SetAngle(Angle newAngle, bool opponentRobot, double angleFrameTime, double newAngleVelocity, bool valid)
{
    // Only do this for our robot
    if (opponentRobot)
    {
        return;
    }

    std::unique_lock<std::mutex> locker(_updateMutex);
    _lastAngle = newAngle;
    _currDataRobot.SetAngle(newAngle, newAngleVelocity, angleFrameTime, valid);
}