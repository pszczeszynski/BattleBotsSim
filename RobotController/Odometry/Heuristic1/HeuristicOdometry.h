#pragma once
#include <opencv2/core.hpp>
#include "HeuristicOdometry.h"
#include "../../ThreadPool.h"
#include "../../Clock.h"
#include "../OdometryBase.h"
#include "RobotTracker.h"
#include <unordered_set>
#include "imgui.h"

// TrackedBBox
// Unlike robot tracker which trackes foreground movement, this track boxes of interest 
// by simply comparing overlap regions. Its faster and good enough for initial filtering

struct TrackedBBox 
{
    myRect bbox;               // Current bounding box
    myRect startBBox;          // Starting bbox
    int frame_count;           // Number of frames the box has existed
    cv::Point2f velocity;      // Velocity in pixels per frame (x, y)
    cv::Point2f last_center;   // Center of the box in the previous frame
    bool is_active;            // Whether the box is still being tracked
    bool is_clear;              // True when the box has in the past moved completly out of its startBBox

    TrackedBBox(myRect _bbox) : bbox(_bbox), startBBox(_bbox), frame_count(1), 
        velocity(0, 0), last_center(_bbox.x + _bbox.width / 2.0f, _bbox.y + _bbox.height / 2.0f), 
        is_active(true), is_clear(false) {}

    TrackedBBox(RobotTracker* tracker);
};


// ***********************
// Camera Decoder Class
extern std::mutex debugROStringForVideo_mutex;
extern std::string debugROStringForVideo;

class HeuristicOdometry : public OdometryBase
{
public:
    HeuristicOdometry(ICameraReceiver *videoSource);

    void processNewFrame(cv::Mat &newFrame);
    void SetPosition(cv::Point2f newPos, bool opponentRobot) override; // Will pick the tracked foreground thats closest to this point
    void ForcePosition(cv::Point2f newPos, bool opponentRobot) override;
    void SwitchRobots(void) override;                                  // Switches who's who
    void SetVelocity(cv::Point2f newVel, bool opponentRobot) override;
    void SetAngle(Angle newAngle, bool opponentRobot, double angleFrameTime, double newAngleVelocity, bool valid) override;
    void GetDebugImage(cv::Mat &target, cv::Point offset = cv::Point(0, 0)) override; // Returns an image that is used for debugging purposes.


    void AutoMatchStart(bool isOurRobotLeft); // Reload background and relocks us to the left most blob, opponent on right
    void MatchStart(bool isOurRobotLeft);     // We're ready, just start

    void RecoverDetection();                // Reload background and relocks us 
    cv::Mat& GetPrevTrackingMatRef(void) { return prevFrameColor; }

    bool enable_camera_antishake = false;  // Turn off/on anti shake
    bool enable_background_healing = true; // Turn off/on background healing
    bool force_background_averaging = false;
    bool useMultithreading = false;

    int averagingCount = 90;                 // Rolling average scaling factor for background areas
    int trackedAvgCount = 200;               // Rolling average scaling factor for untracked bbox areas
    double angleVelocityTimeConstant = 0.2f; // Time constant in seconds for averaging angle velocity

    // Debug
    bool save_to_video_match_debug = false; // tracking robot info dump
    bool save_to_video_output = false;
    bool save_background_to_files = false; // If set to true, will dump background ever 1s
    bool save_background = false;          // If set to true, will dump current background to the file to be loaded next time
    bool load_background = false;           // Loads a new background from saved background  file
    bool load_start_background = true;      // Loads a new background from the specified start background
    bool reinit_bg = false;                // Re-initializes curr background from previous loaded one
    bool set_currFrame_to_bg = false;        // Makes curr frame into background
    bool match_start_bg_init = false;       // Initialize the background using current frame with defined boxes from saved background
    bool show_tuning = false;           // Show foreground extraction stuff
    std::string dumpBackgroundsPath = "backgrounds/dump";
    std::string loadBackgroundsPath = "backgrounds"; // Please to look for preloaded backgrounds
    std::string outputVideoFile = "Recordings/Heuristic_dataDump.mp4";
    int max_distance_to_locate = 100;

    bool show_bg_mat = false;
    bool show_fg_mat = false;
    bool show_track_mat = false;
    bool show_stats = false;
    static cv::Mat backgroundOverlay; // Any overlay we want to apply over the background
    static cv::Size bgOverlaySize;

    static std::string statusstring; // Status string to show in the UI


private:
    void _ProcessNewFrame(cv::Mat currFrame, double frameTime) override; // Run every time a new frame is available
    void _StartCalled() override;
    void _StopCalled() override;                    // Use to destroy all opencv windows
    void _imshow(std::string name, cv::Mat &image); // my own imshow to track the windows we opened
    void _imunshow(std::string name);               // deletes the window if it exists
    std::unordered_set<std::string> cvWindows;

    // Update our data
    void _UpdateData(double timestamp);
    void _UpdateOdometry(OdometryData &data, OdometryData &oldData, RobotTracker *tracker, double timestamp);
    void UpdateSettings(); // Bring in user settings

    Clock clock_outer; // Clock to tell us when to update statistic window (1x per second)

    bool save_video_enabled = false; // Remembers if we were saving video

    // **** Background info
    cv::Mat currBackground;       // The current background to subtract from image (cropped)
    cv::Mat currBackground_16bit; // The same background but 16-bit for averaging
    cv::Mat refBackground;        // The reference background to compare intensities to
    double refIntensities[4] = {-1.0, -1.0, -1.0, -1.0};

    // ***** Color capable currFraem for annotating stuff
    cv::Mat currFrameColor;
    cv::Mat prevFrameColor;

    // Anti-Shake
    int x_offset = 5;             // x offset of camera for cropped imag. Should be >= -1*crop_x and <= crop_x
    int y_offset = 5;             // y offset of camera. Should be >= -1*crop_y and <= crop_y
    int crop_x = 5;               // number of pixels to crop on each side for x. We can scan +/- this amount.
    int crop_y = 5;               // number of pixels to crop on each side for y. We can scan +/- this amount.
    int antiShakeSize = 50;       // The image size to scan for anti-shake
    int x_start_region = 300;     // X Position in the background to start comparing
    int y_start_region = 5;       // Y position in the background to start comparing
    int max_shake_correction = 3; // Only allow up to 3 pixel correction

    // Background healing options
    int robotAvgCount = 900; // NOT USED: Rolling average scaling factor for robot areas

    // Foreground threshold
    cv::Mat foreground; // The masked out foreground image
    cv::Mat fg_mask;    // The mask that defines the foreground

    std::mutex _mutexAllBBoxes; // Mutex to control access to all_bboxes, tracked_bboxes
    std::vector<myRect> all_bboxes; // All the bounding boxes around foreground objects
    std::vector<TrackedBBox> tracked_bboxes; // Persistent list of tracked boxes


    int fg_threshold = 20;            // Minimum intensity difference between background and fg
    double fg_threshold_ratio = 0.1f; // Minimum ratio difference of pixels

    int fg_bbox_minsize = 30;                      // minimum size of a bounding box we want to consider
    int blurCount = 1;                             // Number of times to blur the image
    cv::Size preMaskBlurSize = cv::Size(15, 15);   // Blurring size
    cv::Size fg_mask_blur_size = cv::Size(15, 15); // Blurring size
    int fg_post_blur_threshold = 90;
    int fg_contour_downsize = 2;     // Reduction of resolution when looking at contours
    int fg_contour_bbox_growth = 5;  // Number of pixels on each side to grow bbox to allow them to easily be combined
    int fg_bbox_maxsize = 300; // Maximum dimension of a bbox before we reject it

    // Robot Tracking
    std::vector<RobotTracker *> _allRobotTrackers;
    double currTime = 0.0; // Current frame time
    RobotTracker *ourRobotTracker = NULL;
    RobotTracker *opponentRobotTracker = NULL;
    double deleteForNoMovementTime = 0.7; // Number of s to delete a tracked object for not moving
    int deleteForNoTrackingCount = 150;    // Number of frames to delete a tracked object that hasn't locked on

    // State Machine Variables
    float start_brightness_ok_threshold = 0.7f; // The threshold for brightness to be considered ok at the start of the match
    float brightness_stable_start_time = 0.0; // The time when we started to see stable brightness
    float start_brightness_soak_period = 1.5f; // The time in seconds to wait before we start considering foreground
    bool tracking_started = false; // True if we have started tracking robots
    int clear_margin = 10; // Number of pixels to clear around the tracked bbox to avoid interference with the background healing

    // *********  Functions
    void calcShakeRemoval(cv::Mat &image);
    float correctBrightness(cv::Mat &image, bool no_averaging = false); // Returns the brightness correction factor for the image
    double getBrightnessCorrection(cv::Mat& src, int x, int y, int size , double refMean);
    double correctionValues[4] = {0, 0, 0, 0};
    void healBackground(cv::Mat &currFrame, bool mergeNonRobotAreas = false); // Heals the background by averaging the current frame with the background
    void recreateBackgroundOnMatchStart(cv::Mat& currFrame); // Regenerates background based on the current frame minus tracked fg
    void ReinitBackground();
    bool SaveBackground(void); // Saves the current background to the default file
    bool IsClearOfArea(RobotTracker* tracker, const cv::Rect& area);
    void ResetBrightnessCorrection();

    cv::Mat regularBackground; // Recalled 8-bit grayscale background from file
    void LoadBackground(cv::Mat &currFrame, std::string image_name = "");
    bool DumpBackground(std::string name, std::string path); // Will continueously dump background to files evey 1s
    void ExtractForeground(cv::Mat &croppedFrame);
    RobotTracker *AddTrackedItem(cv::Rect bbox);
    RobotTracker *AddTrackedItem(TrackedBBox tracked);
    void _allRobotTrackersClear(); // clears and deletes all robot trackers
    void DrawAllBBoxes(cv::Mat &mat, int thickness = 1, cv::Scalar scaler = cv::Scalar(255, 0, 0));
    void MatchStartBackgroundInit(cv::Mat &currFrame); // Initializes the match background using the current frame and the defined boxes from the saved background
    float GetForegroundSize(cv::Mat& inBackground, cv::Mat& inForeground); // Returns the average value of the fg mask
    float FindOptimalBrightness(cv::Rect& myRect, cv::Mat& currFrame);

    float CalculateIoU(const myRect& box1, const myRect& box2); // Returns the bbox union area as a fraction of total combined area.
    bool IsSimilarSize(const myRect& box1, const myRect& box2, float max_size_error = 0.3f); // Returns true if boxes are similar in size


    std::mutex _mutexTrackData; // Used for parallel processes to report back they are done with the conditional variable below
    std::condition_variable_any conditionVarTrackRobots; 
    

    void TrackRobots(cv::Mat &croppedFrame, cv::Mat &frameToDisplay);
    bool LocateRobots(cv::Point2f newPos, bool opponentRobot, bool skipMutexLock = false);
    void _deleteTracker(RobotTracker *staleTracker);

    // Video Saving for debugging
    cv::VideoWriter video;
    double time_to_stop_video = 120.0;
    void SaveToVideo(cv::Mat &image);
    void DumpRobotTrackerInfo(cv::Mat &channel, std::string title);
    void DumpCurrFrameInfo(cv::Mat &channel, cv::Mat &currForeground, std::string title);
    void AddDebugStringToFrame(cv::Mat &channel, std::string debugString);

    std::mutex _mutexDebugImage;
    cv::Mat _debugImage;
};
