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
#define MAX_EXTRAPOLATION_TIME_S 0.1

class OdometryData
{
public:
    OdometryData(int id = 0);

    int id = 0; // Increment ID whenever data changes. A value of 0 means it
                // hasn't been initialized yet
    int frameID = -1; // The ID of the last video frame this data is based off.
                      // A value of -1 means it hasn't been initialized yet

    double time = 0; // The time of the last video frame this data is based off
                     // since the start of this program in seconds. A value of 0
                     // means it hasn't been initialized yet
    void Clear();    // Clears all position and user data to invalid;

    bool IsAngleValid(); // returns true if the angle is valid
    void SetAngle(Angle newAngle, double newAngleVelocity,
                  double angleFrameTime, bool valid);
    Angle GetAngle(); // gets the last set angle without extrapolation
    double GetAngleFrameTime();  // returns the time of the frame the angle was calculated on
    double GetAngleVelocity(); // returns the last set angle velocity

    // returns a new instance of the data extrapolated to the target time
    // the maxRelativeTime is the maximum time to extrapolate forward
    OdometryData ExtrapolateBoundedTo(double targetTime,
                                      double maxRelativeTime = MAX_EXTRAPOLATION_TIME_S);

    void InvalidatePosition();
    void InvalidateAngle();

    // Set to true for our robot to help generic functions know
    bool isUs = false;

    // Our Position
    bool robotPosValid = false;
    cv::Point2f robotPosition;
    cv::Point2f robotVelocity; // Assume if position is good velocity is good, just starts at 0 if first frame

    // The rectangle to draw around us. only valid if robotPosValid is true
    // This should be optional
    cv::Rect rect;


    // User data for tracking algorithm internals
    std::unordered_map<std::string, double> userDataDouble;


    double GetAge(); // returns the age of this data in seconds

    bool IsPointInside(cv::Point2f point)
    {
        return (robotPosValid && rect.contains(point));
    }

    void GetDebugImage(cv::Mat& target, cv::Point offset = cv::Point(0, 0)); // Returns an image that is used for debugging purposes.
private:
    // extrapolates the data to the new time without bound!
    OdometryData _ExtrapolateTo(double newtime);

    // Rotation
    bool _angleValid = false;
    double _angleVelocity = 0; // Clockwise
    Angle _angle;
    double _angleFrameTime = -1; // The time of the last angle update
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
    virtual void SetPosition(cv::Point2f newPos, bool opponentRobot);  // Sets position: Odomotry will try to find the closest BBOX to meet this
    virtual void ForcePosition(cv::Point2f newPos, bool opponentRobot); // Forces position to be overriden regardless of state 
    virtual void SetVelocity(cv::Point2f newVel, bool opponentRobot);
    virtual void SetAngle(Angle newAngle, bool opponentRobot,
                          double angleFrameTime, double newAngleVelocity,
                          bool valid);

    virtual void GetDebugImage(cv::Mat& target, cv::Point offset = cv::Point(0, 0)); // Returns an image that is used for debugging purposes. It can be empty if no debug image is available

protected:
    // If not overriding Start/Stop/IsRunning, then you can tie into this:
    virtual void _StartCalled(void){};                                    // Run when a new thread is started
    virtual void _ProcessNewFrame(cv::Mat currFrame, double frameTime){ }; // Run every time a new frame is available. currFrame is a copy.
    virtual void _StopCalled(){};                                         // Run when stop is initiated

    ICameraReceiver *_videoSource = nullptr;

    long frameID = -1;

    // Core thread
    std::thread processingThread; // Main thread of the algorithm
    std::atomic<bool> _running = false;        // This marks whether the processingThread is running
    std::atomic<bool> _stopWhenAble = false;   // This requests that the main process stop.

    // Data that stores results
    std::mutex _updateMutex;        // Mutex for updating core results
    OdometryData _currDataRobot;    // Contains the current data for us
    OdometryData _currDataOpponent; // Contains the current data for opponent. TODO: May want this to be an array to track multiple opponents
};
