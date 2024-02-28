#include "OdometryIMU.h"
#include "../../UIWidgets/ImageWidget.h"
#include "../../RobotConfig.h"
#include "../../RobotController.h"

OdometryIMU::OdometryIMU() : OdometryBase(nullptr)
{
}

// Start the thread
// Returns false if already running
bool OdometryIMU::Run(void)
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

        while(_running && !_stopWhenAble)
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

void OdometryIMU::_UpdateData(IMUData &imuData, double timestamp)
{
    // Get unique access
    std::unique_lock<std::mutex> locker(_updateMutex);

    _currDataRobot.id++;                // Increment frame id
    _currDataRobot.time = timestamp;    // Set to new time
    _currDataOpponent.id++;             // Increment frame id
    _currDataOpponent.time = timestamp; // Set to new time

    // Clear curr data
    _currDataRobot.Clear();
    _currDataRobot.isUs = true; // Make sure this is set
    _currDataOpponent.Clear();
    _currDataOpponent.isUs = false; // Make sure this is set

    // Set our rotation
    _currDataRobot.robotAngleValid = true;
    _currDataRobot.robotAngle = Angle(imuData.rotation + _angleOffset);
    _currDataRobot.robotAngleVelocity = imuData.rotationVelocity;

    _lastIMUAngle = imuData.rotation;
}

void OdometryIMU::SetAngle(double newAngle, bool opponentRobot)
{
    // Only do this for our robot
    if (opponentRobot)
    {
        return;
    }

    // Don't need mutex acces to angleOffset;
    _angleOffset = newAngle - _lastIMUAngle;
}