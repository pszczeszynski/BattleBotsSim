#pragma once
#include "../Clock.h"
#include <signal.h>
#include <windows.h>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include "../MathUtils.h"
#include "../CameraReceiver.h"
#include "../VisionPreprocessor.h"
#include <string.h>
#include <unordered_map>

// The total time to wait stopping the main task in seconds before killing it
#define ODOMETRY_STOP_TIMEOUT 0.1f
#define ODO_MUTEX_TIMEOUT std::chrono::milliseconds(250)

class OdometryData
{
public:
    OdometryData();

    int id = 0; // Increment ID whenever data changes. A value of 0 means it hasn't been initialized yet

    double time = 0; // The time of the last video frame this data is based off since the start of this program in seconds.
                     // A value of 0 means it hasn't been initialized yet

    bool isUs = false; // Set to true for our robot to help generic functions know

    // Our Position
    bool robotPosValid = false;
    cv::Point2f robotPosition;
    cv::Point2f robotVelocity; // Assume if position is good velocity is good, just starts at 0 if first frame

    // The rectangle to draw around us. only valid if robotPosValid is true
    // This should be optional
    cv::Rect rect;

    // Our Rotation
    bool robotAngleValid = false;
    Angle robotAngle;
    double robotAngleVelocity = 0; // Clockwise

    // User data for tracking algorithm internals
    std::unordered_map<std::string, double> userDataDouble;

    void Clear(); // Clears all position and user data to invalid;

    void Extrapolate(double newtime); // Extrapolates position and anlge into newtime future
};

// ***********************
// Base Odometry API definition
class OdometryBase
{
public:
    // The Odometry algorithm is expected to run a parallel thread that gets new video
    // frames from the videoSource as often as it wants.
    // The processing of the image for birds eye view is thus going to be done by the videoSource
    OdometryBase(ICameraReceiver *videoSource);
    OdometryBase(void){};

    virtual bool NewDataValid(int oldId, bool getOpponent = false); // Returns true if new data is present
    virtual OdometryData GetData(bool getOpponent = false);         // Returns latest OdometryData. False = us, true = opponent

    virtual bool Run(void);       // Starts the thread(s) to decode data. Returns true if succesful
    virtual bool Stop(void);      // Stops the thread(s) if running. Returns true if succesful or if thread wasnt running.
    virtual bool IsRunning(void); // Returns true if the algorithm is running

    virtual void SwitchRobots(void); // Switches who's who
    virtual void SetPosition(cv::Point2f newPos, bool opponentRobot);
    virtual void SetVelocity(cv::Point2f newVel, bool opponentRobot);
    virtual void SetAngle(double newAngle, bool opponentRobot);

protected:
    // If not overriding Start/Stop/IsRunning, then you can tie into this:
    virtual void _StartCalled(void){};                                    // Run when a new thread is started
    virtual void _ProcessNewFrame(cv::Mat currFrame, double frameTime){}; // Run every time a new frame is available. currFrame is a copy.
    virtual void _StopCalled(){};                                         // Run when stop is initiated

    ICameraReceiver *_videoSource = nullptr;

    // Core thread
    std::thread processingThread; // Main thread of the algorithm
    bool _running = false;        // This marks whether the processingThread is running
    bool _stopWhenAble = false;   // This requests that the main process stop.

    // Data that stores results
    std::mutex _updateMutex;        // Mutex for updating core results
    OdometryData _currDataRobot;    // Contains the current data for us
    OdometryData _currDataOpponent; // Contains the current data for opponent. TODO: May want this to be an array to track multiple opponents
};
