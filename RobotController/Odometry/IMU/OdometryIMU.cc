#include "OdometryIMU.h"
#include "../../UIWidgets/ImageWidget.h"
#include "../../RobotConfig.h"

OdometryIMU::OdometryIMU(RobotController &controller) : _controller(controller)
{

}

// Start the thread
// Returns false if already running
bool OdometryIMU::Run( void )
{
    if( _running )
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
            frameID = _controller.GetIMUFrame( newMessage, frameID, &frameTime); // Blocking read until new frame available

            // Update the data
            _UpdateData( newMessage, frameTime);
        }


        // Exiting thread
        _running = false;
        _stopWhenAble = false;           
    });

    return true;
}


void  OdometryIMU::_UpdateData(IMUData& imuData, double time)
{
    // TBD TBD TBD TBD
}