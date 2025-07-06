#pragma once
#include <opencv2/core.hpp>
#include "../../ThreadPool.h"
#include <math.h>
#include "../../Clock.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef deg2rad
#define deg2rad(deg) (deg * M_PI / 180.0)
#endif

#ifndef rad2deg
#define rad2deg(rad) (rad * 180.0 / M_PI)
#endif

#ifndef angleWrap
#define angleWrap(degree) ((degree < -180.0) ? fmod(degree + 180.0, 360.0) + 180.0 : fmod(degree + 180.0, 360.0) - 180.0)
#endif

#ifndef angleWrap90
#define angleWrap90(degree) ((degree < -90.0) ? fmod(degree + 90.0, 180.0) + 90.0 : fmod(degree + 90.0, 180.0) - 90.0)
#endif

#ifndef angleWrapRad
#define angleWrapRad(rad) ((rad < -M_PI) ? fmod(rad + M_PI, 2*M_PI) + M_PI : fmod(rad + M_PI, 2*M_PI) - M_PI)
#endif

class Vector2;
class myRect;

// ******************************
// General purpose function
int FindBBoxWithLargestOverlap(const std::vector<myRect> &allBBoxes, const cv::Rect &targetBBox, cv::Rect &bestBBox, int &indexFound);
float FindClosestBBox(const std::vector<myRect> &allBBoxes, const cv::Point2f &point, cv::Rect &bestBBox, int &indexFound);
void printText(std::string text, cv::Mat &image, int yoffset = 50, int xoffset = 50);
cv::Rect FixBBox(const cv::Rect &bbox, const cv::Mat &mat); // Retuns a bbox that is clamped to the image
cv::Rect FixBBox(const cv::Rect &bbox, const cv::Size &matSize);
cv::Point2f GetRectCenter(const cv::Rect &inbox);
cv::Point2f LimitPointToRect(cv::Rect &roi, cv::Point2f point);
Vector2 LimitPointToRect(cv::Rect &roi, Vector2 point);
cv::Rect getBoundingRect(const cv::Rect &rect1, const cv::Rect &rect2, int buffer = 0);
cv::Size RotateSize(cv::Size startSize, float degrees);
cv::Rect MoveRectToBeInside(cv::Rect recttofix, cv::Mat &mat_to_fit);
cv::Rect CalculateMaskBBox(const cv::Mat &mask);
cv::Rect CropRect(cv::Rect ref_rect, cv::Rect croping);
bool DoRectOverlap(const cv::Rect& rect1, const cv::Rect& rect2);

// Debugging variables
extern Clock timing_clock;
extern std::vector<double> timing_list;
extern std::vector<std::string> timing_text;
void markTime(std::string note = "Time:", double time = -1);


// ***************************
// Vector2 Class
class Vector2
{
public:
    float x = 0;
    float y = 0;

    float mag();
    float angle();    // angle in degrees
    float angleRad(); // angle in radians
    Vector2(float x = 0, float y = 0) : x(x), y(y) {}
    cv::Point Point(float scale = 1.0);
    cv::Point2f Point2f();

    // Assignment operator for cv::Point2f
    Vector2& operator=(const cv::Point2f& point) {
        this->x = point.x;
        this->y = point.y;
        return *this;
    }

    // Copy assignment operator
    Vector2& operator=(const Vector2& other) {
        if (this != &other) { // prevent self-assignment
            x = other.x;
            y = other.y;
        }
        return *this;
    }

    // Multiplication operator 
    Vector2 operator*(const float& scalar) const {
        return Vector2(x * scalar, y * scalar);
    }   

    // *= operator
    Vector2& operator*=(const float& scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    // Subtraction operator
    Vector2 operator-(const Vector2& other) const {
        return Vector2(x - other.x, y - other.y);
    }

    // -= operator
    Vector2& operator-=(const Vector2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    // Addition operator
    Vector2 operator+(const Vector2& other) const {
        return Vector2(x + other.x, y + other.y);
    }

    // += operator
    Vector2& operator+=(const Vector2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
};

// myRect class
class myRect : public cv::Rect
{
public:
    int numOfOwners = 0;

    myRect() : cv::Rect()
    {
        numOfOwners = 0;
    }

    myRect(int _x, int _y, int _width, int _height)
        : cv::Rect(_x, _y, _width, _height)
    {
        numOfOwners = 0;
    }

    myRect(const cv::Rect &refrect)
        : cv::Rect(refrect)
    {
        numOfOwners = 0;
    }

    cv::Point2f Center()
    {
        return cv::Point2f( x+width/2.0f, y+height/2.0f );
    }

    // Overload multiplication operator for intersection of two myRect objects
    myRect operator*(const myRect& other) const
    {
        myRect result = *this & other; // Use cv::Rect's intersection operator
        result.numOfOwners = 0; // Reset owners
        return result;
    }
};

// ***********************
// Robot Tracker Class
class RobotTracker
{
public:
    // Settings initialized in RobotTracker.cc
    static bool useMultithreading;
    static int numberOfThreads;
    static int maxNewBBoxIncrease;                // Number of pixels new box can grow by
    static float robotVelocitySmoothing;          // (s) smoothing of the velocity projection
    static float detectionVelocitySmoothing;      // (s) smoothing of the background detection
    static float minVelocity;                     // Min number of pixels/s movement required
    static float moveTowardsCenter;               // Amount of pixels per second to move towards center (?20?)
    static float bboxFoundIsMuchSmallerThreshold; // The area reduction in bounding box that will trigger regeneration of Foreground
    static int combinedBBoxScanBuffer;            // Number of pixels to grow extrapolated bbox by to scan a combined bbox with
    static bool matchingAreaAddOurBBoxes;         // increase matching area to always include our bbox and predicted bbox
    static bool derateResultsByDistance;
    static float distanceDerating_start;    // The mutliplier to results at the expected location
    static float distanceDerating_stop;     // The maximum derating for things far away
    static float distanceDerating_distance; // The radial distance at which we reach _stop intensity
    static int distanceDeratingMatSize;     // Width/Height. Large enough to ensure we never run out. Most we will every use is mayb 100x100.
    static int ageBeforeStartInit;
    static bool enableOlderImageTracking;   // Uses older image if not much movement/rotation occurs to have more accurate rotation. Not very effective at this time.

    // Find Pos and Rotation using Match TemplateSearch. This will be done using parallel operation
    static float deltaAngleSweep; // Delta +/- angle to sweep
    static float deltaAngleStep;  // Number of degrees to step angle sweep
    static int matchBufffer;      // number of pixels around all edges to expand search (fixed pixels)

    int id = -1; // Unique ID

    // Tracking Info
    int numFramesOld = 0; // Number of frames this tracker has been active
    bool lockedOn = false;
    float trackingError = 0;
    int numFramesNotTracked = 0;
    double timeNotMoving = 0;
    bool debug_DumpInfo = false;                 // Enable dumping info to debugImage
    double debug_timeToSnapRotation = 30.5;      // Time in (s)
    double debug_timeToSnapRotationduration = 5; // Time in (s)
    std::string debugLine = "";

    cv::Rect bbox;    // Position of the fg_image on the screen
    cv::Mat fg_image; // The foreground image contained in bbox
    cv::Mat fg_mask;  // the foreground mask contained in bbox
    cv::Mat debugImage;
    cv::Mat matchResultSaved;

    double lastTime = 0;             // Time of last velocity/position update
    double currTimeSaved = 0;        // The current time passed to the function
    Vector2 position;                // Not necessarily the center of the bounding box, also supports fractions for movement filtering
    Vector2 rotation;                // clockwise rotation in 2D vector
    Vector2 avgVelocity;             // Velocity for BBox projection
    Vector2 currVelocity;            // Instantanouse velocity
    Vector2 velForMovementDetection; // Very slow filtered velocity
    cv::Point delta;                 // Delta position of the frame
    float delta_angle = 0;           // The frame delta angle

    // Settings

    cv::Mat distanceDerating; // The scalling mat applied to results to give higher weight to expected positions

    void InitializeDeratingMat(void);
    cv::Rect correctResultsForDistance(cv::Mat &results, const cv::Rect &scannedBox, const cv::Mat &templ, const cv::Point2f &templ_center);

    RobotTracker(cv::Rect bbox);
    cv::Rect GetExtrapolatedBBOX(double curr_time);
    Vector2 GetExtrapolatedPosition(double curr_time);
    void ProcessNewFrame(double currTime, cv::Mat &foreground, cv::Mat &currFrame, cv::Mat &new_fg_mask, int &doneInt, std::condition_variable_any &doneCV, std::mutex &mutex, cv::Mat &debugMat); // Porcess new frame, return true if lock occured
    
    cv::Point GetCenter(void);
    void SetRotation(double angleRad);
    bool IsPointInside(cv::Point2i point);
    bool IsTrackerCombined(void);

    cv::Rect startBbox;
    bool clearedStart = false; // True if we cleared the start bbox
    bool startInitialized = false; // True if we initialized the start bbox


    cv::Rect predictedBBox;
    myRect *bestBBox = nullptr;
    void FindBestBBox(double currTime, std::vector<myRect> &allBBoxes); // Finds and saves best bbox

    double FindNewPosAndRotUsingMatchTemplate(cv::Mat &currFrame, cv::Mat &foreground, cv::Rect &fgFoundBBox, Vector2 &newRot);

    std::mutex mutexResults;
    std::condition_variable_any conditionVarResults;
    double sweep_minVal = -1, sweep_maxVal = -1, sweep_maxVal_old = 1;
    float sweep_rotation = -180;
    int processes_done = 0;
    double delta_confidence = 0; // maxVal new ratio to old, if <1 by a lot means low confidence
    cv::Point sweep_minLoc, sweep_maxLoc;
    cv::Mat finalImage;
    cv::Mat finalMask;
    cv::Rect finalCorrectionROI;
    cv::Rect finalMatchingROI;

    cv::Rect finalBBox; // Referenced to top-left of scanned image (matchingBBox)
    void matchTemplateThread(const cv::Rect &matLocation, const cv::Mat &matchingMat, float currAngleStart, float currAngleStop);

    void FixPartialForeground(cv::Mat &currFrame, cv::Mat &foreground, cv::Mat &new_fg_mask, cv::Rect &bestBBox);

    // New code for keeptrick track of older image for drift reduction
    // New member variables for older state
    cv::Mat old_fg_image;      // Older foreground image
    cv::Mat old_fg_mask;       // Older foreground mask
    cv::Rect old_bbox;         // Older bounding box
    Vector2 old_position;      // Older position
    Vector2 old_rotation;      // Older rotation
    double old_time;           // Time of older state
    bool old_state_valid;      // Flag to indicate if older state is valid

    // New static thresholds
    static float max_rotation_diff;    // Maximum rotation difference (degrees) to trigger update
    static float maxTimeToNotUpdateRotImage;
    static float maxMovementToNotUpdateRotImage;

};