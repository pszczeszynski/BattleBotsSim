#pragma once
#include <thread>
#include <array>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include "../CameraReceiver.h"
#include "OdometryData.h"

// The total time to wait stopping the main task in seconds before killing it
#define ODOMETRY_STOP_TIMEOUT 0.1f
#define ODO_MUTEX_TIMEOUT std::chrono::milliseconds(250)

enum class RobotSlot { Us = 0, Opponent = 1 };

// ***********************
// Base Odometry API definition
class OdometryBase
{
public:
    // The Odometry algorithm is expected to run a parallel thread that gets new video
    // frames from the videoSource as often as it wants.
    // The processing of the image for birds eye view is thus going to be done by the videoSource
    OdometryBase(ICameraReceiver *videoSource);
    OdometryBase(void);

    virtual bool HasNewerDataById(int oldId, bool getOpponent = false); // Returns true if there's newer data than the old id
    virtual OdometryData GetData(bool getOpponent = false);         // Returns latest OdometryData. False = us, true = opponent

    virtual bool Run(void);       // Starts the thread(s) to decode data. Returns true if succesful
    virtual bool Stop(void);      // Stops the thread(s) if running. Returns true if succesful or if thread wasnt running.
    virtual bool IsRunning(void); // Returns true if the algorithm is running

    virtual void SwitchRobots(void); // Switches who's who
    virtual void SetPosition(cv::Point2f newPos, bool opponentRobot);  // Sets position: Odomotry will try to find the closest BBOX to meet this
    virtual void ForcePosition(cv::Point2f newPos, bool opponentRobot); // Forces position to be overriden regardless of state 
    virtual void SetVelocity(cv::Point2f newVel, bool opponentRobot);
    virtual void SetAngle(AngleData angleData, bool opponentRobot);

    virtual void GetDebugImage(cv::Mat& target, cv::Point offset = cv::Point(0, 0)); // Returns an image that is used for debugging purposes. It can be empty if no debug image is available

protected:
    // If not overriding Start/Stop/IsRunning, then you can tie into this:
    virtual void _StartCalled(void){};                                    // Run when a new thread is started
    virtual void _ProcessNewFrame(cv::Mat currFrame, double frameTime){ }; // Run every time a new frame is available. currFrame is a copy.
    virtual void _StopCalled(){};                                         // Run when stop is initiated

    // Centralized publish function - algorithms build a sample and call this
    void Publish(OdometryData sample, bool isOpponent);

    ICameraReceiver *_videoSource = nullptr;

    long frameID = -1;

    // Core thread
    std::thread processingThread; // Main thread of the algorithm
    std::atomic<bool> _running = false;        // This marks whether the processingThread is running
    std::atomic<bool> _stopWhenAble = false;   // This requests that the main process stop.

    // Data that stores results
    std::mutex _updateMutex;        // Mutex for updating core results
    std::array<OdometryData, 2> _data;  // [0] = Us, [1] = Opponent
};
