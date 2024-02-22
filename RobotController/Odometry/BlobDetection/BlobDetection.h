#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "../../Globals.h"
#include "../OdometryBase.h"
#include "RobotClassifier.h"
#include "../../CameraReceiver.h"
#include "../../VisionClassification.h"
#include "MotionBlob.h"



// BlobDetection Odometry
// Tracks objects based on the blob movement
class BlobDetection : public OdometryBase
{
public: 
    BlobDetection(ICameraReceiver &videoSource);

    void SwitchRobots( void ) override;
    void SetPosition(cv::Point2f newPos, bool opponentRobot) override;
    void SetVelocity(cv::Point2f newVel, bool opponentRobot) override;
    void SetAngle(double newAngle, bool opponentRobot) override;

private:
    void _ProcessNewFrame(cv::Mat currFrame, double frameTime) override;  // Run every time a new frame is available

    VisionClassification DoBlobDetection(cv::Mat& currFrame, cv::Mat& prevFrame);           // The core blob detection algorithm
    void UpdateData( VisionClassification robotData, double timestamp); // Updates the core data so others can poll it

    bool _IsValidBlob(MotionBlob &blobNew, OdometryData& currData, OdometryData& prevData ); // Checks if blob is valid
    void _GetSmoothedVisualVelocity(OdometryData& currData, OdometryData& prevData); // Averages velocity since its comming in jittery
    void UpdateVisionOnly(MotionBlob* blob, OdometryData& currData, OdometryData& prevData);
    void CalcAnglePathTangent(OdometryData& currData, OdometryData& prevData);
   
    RobotClassifier _robotClassifier; // Takes the blobs and figures out whos who
    cv::Mat _previousImage;      // The previous image to do delta on
    Clock _prevImageTimer;      // Keeps track of when we updated our previous image
    double prevFrameTime = 0;   // The previous time for velocity calcs


    cv::Mat _lastDrawingImage;  // Copy of the last image for other processes to pull
    OdometryData _prevDataRobot;     // Contains the previous data, useful for velocity calculations, etc..
    OdometryData _prevDataOpponent;     // Contains the previous data, useful for velocity calculations, etc..

};









/*
    static BlobDetection& Robot();
    static BlobDetection& Opponent();

    void UpdateVisionAndIMU(MotionBlob& blob, cv::Mat& frame);
    double UpdateForceSetAngle(double newAngle);
    void UpdateIMUOnly(cv::Mat& frame);
    void UpdateVisionOnly(MotionBlob& blob, cv::Mat& frame);

    // used for calibration
    void UpdateForceSetPosAndVel(cv::Point2f position, cv::Point2f velocity);

    void Invalidate();

    cv::Point2f GetPosition();
    Angle GetAngle();


    cv::Point2f GetVelocity();
    double GetAngleVelocity();

    void InvertAngle();
*/


/*
    bool _IsValidBlob(MotionBlob& blob);
    double _lastBlobArea;
    int _numUpdatesInvalid;

    void _PostUpdate(cv::Point2f position, cv::Point2f velocity, Angle angle);
    double _UpdateAndGetIMUAngle();
    cv::Point2f _GetSmoothedVisualVelocity(MotionBlob& blob);

    double _lastIMUAngle;

    Angle _angle;
    cv::Point2f _position;
    bool _isValid;

    Angle CalcAnglePathTangent();
    bool _visualAngleValid = false;
    Clock _lastVisualAngleValidClock;

    cv::Point2f _lastPositionWhenUpdatedAngle; // angle updated based on displacement

    cv::Point2f _lastVelocity;
    double _angleVelocity;

    Clock _lastUpdateClock;
    Clock _lastVelocityCalcClock;

    Clock _lastAccelIntegrateClock;
*/

/*
    #define VISUAL_VELOCITY_HISTORY_SIZE 10
    std::deque<cv::Point2f> _visualVelocities;
};
*/
