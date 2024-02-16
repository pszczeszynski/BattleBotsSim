#pragma once
#include <opencv2/core.hpp>
#include "CameraReceiver.h"
#include "VisionPreprocessor.h"
#include "ThreadPool.h"
#include <math.h>



#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef deg2rad
#define deg2rad(deg)  (deg*M_PI/180.0)
#endif

#ifndef rad2deg
#define rad2deg(rad)  (rad*180.0/M_PI)
#endif

#ifndef angleWrap
#define angleWrap(degree) ((degree<-180.0) ? fmod(degree+180.0,360.0)+180.0 : fmod(degree+180.0,360.0)-180.0)
#endif

class Vector2;
class myRect;

// ******************************
// General purpose function
int FindBBoxWithLargestOverlap(const std::vector<myRect>& allBBoxes, const cv::Rect& targetBBox,  cv::Rect& bestBBox,int& indexFound  );
void printText(std::string text, cv::Mat& image, int yoffset=50, int xoffset=50);
extern ThreadPool myThreads; // Will initialize to maximum threads it can
cv::Rect FixBBox(const cv::Rect& bbox, const cv::Mat& mat); // Retuns a bbox that is clamped to the image
cv::Rect FixBBox(const cv::Rect& bbox, const cv::Size& matSize); 
cv::Point2f GetRectCenter(cv::Rect& inbox);
cv::Point2f LimitPointToRect( cv::Rect& roi, cv::Point2f point);
Vector2     LimitPointToRect( cv::Rect& roi, Vector2 point);
cv::Rect getBoundingRect(const cv::Rect& rect1, const cv::Rect& rect2, int buffer=0);
cv::Size RotateSize(cv::Size startSize, float degrees);
cv::Rect MoveRectToBeInside(cv::Rect recttofix, cv::Mat& mat_to_fit);
cv::Rect CalculateMaskBBox(const cv::Mat& mask);
cv::Rect CropRect(cv::Rect ref_rect, cv::Rect croping);

// Debugging variables
extern Clock timing_clock;
extern std::vector<double> timing_list;
extern std::vector<std::string> timing_text;
void markTime(std::string note = "Time:", double time = -1);

// This class only does something when it is destroyed
// It locks the mutext, increments the count and unlocks the mutex.
// It also notifies_all the conditional variable when it is destroyed.
class MultiThreadCleanup
{
public:
    MultiThreadCleanup(int& count_to_increment, std::mutex& mutex, std::condition_variable_any& cv);
    ~MultiThreadCleanup();

private:
    int& count_to_increment_;
    std::condition_variable_any& cv_;
    std::mutex& mutex_;
};


// ***************************
// Vector2 Class
class Vector2
{
public:
    float x=0;
    float y=0;

    float mag();
    float angle(); // angle in degrees
    Vector2(float x=0, float y=0) : x(x),y(y) {}
    cv::Point Point(float scale = 1.0);
};


// myRect class
class myRect : public cv::Rect
{
public:
    int numOfOwners=0;

    myRect() : cv::Rect() {
        numOfOwners = 0;
    }

    myRect(int _x, int _y, int _width, int _height) 
        : cv::Rect(_x, _y, _width, _height) {
            numOfOwners = 0;
    }
    
    myRect(const cv::Rect& refrect) 
        : cv::Rect(refrect) {
            numOfOwners = 0;
    }
};

// ***********************
// Robot Tracker Class
class RobotTracker
{
public:
    int id = -1; // Unique ID 

    // Tracking Info
    bool lockedOn = false;
    float trackingError = 0;
    int numFramesNotTracked = 0;
    double timeNotMoving = 0;
    bool debug_DumpInfo = false; // Enable dumping info to debugImage
    double debug_timeToSnapRotation = 30.5; // Time in (s)
    double debug_timeToSnapRotationduration = 5; // Time in (s)
    std::string debugLine = "";

    cv::Rect bbox; // Position of the fg_image on the screen
    cv::Mat fg_image; // The foreground image contained in bbox
    cv::Mat fg_mask; // the foreground mask contained in bbox
    cv::Mat debugImage;
    cv::Mat matchResultSaved;

    double lastTime = 0; // Time of last velocity/position update
    double currTimeSaved = 0; // The current time passed to the function
    Vector2 position; // Not necessarily the center of the bounding box, also supports fractions for movement filtering
    Vector2 rotation; //clockwise rotation in 2D vector
    Vector2 avgVelocity; // Velocity for BBox projection
    Vector2 currVelocity; // Instantanouse velocity
    Vector2 velForMovementDetection; // Very slow filtered velocity
    cv::Point delta; // Delta position of the frame
    float delta_angle =0; // The frame delta angle

    // Settings
    int maxNewBBoxIncrease = 40; // Number of pixels new box can grow by
    float robotVelocitySmoothing = 0.2; // (s) smoothing of the velocity projection
    float detectionVelocitySmoothing = 1.0; // (s) smoothing of the background detection
    float minVelocity = 7; // Min number of pixels/s movement required
    float moveTowardsCenter = 40; // Amount of pixels per second to move towards center (?20?)
    float rotateTowardsMovement = 0; // Amount of radians per second to move towards velocity dir (?2?)
    float rotateTowardsWeight = 1.0/50.0; // Scaling factor to increase weight vs speed 
    float minSpeedForRotationCorrection = 30.0; // Minimum speed in pixels/s before we add in movement
    float bboxFoundIsMuchSmallerThreshold = 0.75; // The area reduction in bounding box that will trigger regeneration of Foreground
    int combinedBBoxScanBuffer = 15; // Number of pixels to grow extrapolated bbox by to scan a combined bbox with
    bool matchingAreaAddOurBBoxes = true; // increase matching area to always include our bbox and predicted bbox
    bool derateResultsByDistance = true;
    cv::Mat distanceDerating; // The scalling mat applied to results to give higher weight to expected positions
    float distanceDerating_start = 1.0; // The mutliplier to results at the expected location
    float distanceDerating_stop = 0.5; // The maximum derating for things far away
    float distanceDerating_distance = 15.0; // The radial distance at which we reach _stop intensity
    int distanceDeratingMatSize = 999; // Width/Height. Large enough to ensure we never run out. Most we will every use is mayb 100x100.
    void InitializeDeratingMat(void);
    cv::Rect correctResultsForDistance(cv::Mat& results, const cv::Rect& scannedBox, const cv::Mat& templ, const cv::Point2f& templ_center );


    RobotTracker(cv::Rect bbox);
    cv::Rect GetExtrapolatedBBOX(double curr_time);
    Vector2 GetExtrapolatedPosition(double curr_time);
    void ProcessNewFrame(double currTime,  cv::Mat& foreground,  cv::Mat& currFrame,  cv::Mat& new_fg_mask, int& doneInt, std::condition_variable_any& doneCV, std::mutex& mutex, cv::Mat& debugMat );//Porcess new frame, return true if lock occured

    cv::Point GetCenter(void);

    // Find Pos and Rotation using Match TemplateSearch. This will be done using parallel operation
    float deltaAngleSweep = 15; // Delta +/- angle to sweep
    float deltaAngleStep = 1; // Number of degrees to step angle sweep
    int matchBufffer = 10; // number of pixels around all edges to expand search (fixed pixels)
   
    cv::Rect predictedBBox;
    myRect* bestBBox;
    void FindBestBBox(double currTime, std::vector<myRect>& allBBoxes ); // Finds and saves best bbox

    double FindNewPosAndRotUsingMatchTemplate( cv::Mat& currFrame,  cv::Mat& foreground, cv::Rect& fgFoundBBox, Vector2& newRot);

    bool useMultithreading = true;
    float numberOfThreads = 8.0;
    std::mutex mutexResults;
    std::condition_variable_any conditionVarResults;
    double sweep_minVal=-1, sweep_maxVal=-1, sweep_maxVal_old=1;
    float sweep_rotation = -180;
    int processes_done = 0;
    double delta_confidence = 0; // maxVal new ratio to old, if <1 by a lot means low confidence
    cv::Point sweep_minLoc, sweep_maxLoc;
    cv::Mat finalImage;
    cv::Mat finalMask;
    cv::Rect finalCorrectionROI;
    cv::Rect finalMatchingROI;

    cv::Rect finalBBox; // Referenced to top-left of scanned image (matchingBBox)
    void matchTemplateThread(const cv::Rect& matLocation, const cv::Mat& matchingMat, float currAngleStart, float currAngleStop);

    void FixPartialForeground(cv::Mat& currFrame, cv::Mat& foreground, cv::Mat& new_fg_mask, cv::Rect& bestBBox);

};