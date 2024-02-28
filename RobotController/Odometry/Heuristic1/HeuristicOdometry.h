#pragma once
#include <opencv2/core.hpp>
#include "HeuristicOdometry.h"
#include "../../ThreadPool.h"
#include "../../Clock.h"
#include "../OdometryBase.h"
#include "RobotTracker.h"
#include <unordered_set>

// ***********************
// Camera Decoder Class
class HeuristicOdometry : public OdometryBase
{
public:
    HeuristicOdometry(ICameraReceiver *videoSource);

    void processNewFrame(cv::Mat &newFrame);
    void SetPosition(cv::Point2f newPos, bool opponentRobot) override; // Will pick the tracked foreground thats closest to this point
    void SwitchRobots(void) override;                                  // Switches who's who
    void SetVelocity(cv::Point2f newVel, bool opponentRobot) override;
    void SetAngle(double newAngle, bool opponentRobot) override;

    void MatchStart(cv::Point2f robotPos, cv::Point2f opponentPos); // Reload background and relocks us to the left most blob, opponent on right

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
    bool load_background = true;           // Loads a new background from file
    bool reinit_bg = false;                // Re-initializes curr background from previous loaded one
    std::string dumpBackgroundsPath = "backgrounds/dump";
    std::string loadBackgroundsPath = "backgrounds"; // Please to look for preloaded backgrounds
    std::string outputVideoFile = "Recordings/Heuristic_dataDump.mp4";

    bool show_bg_mat = false;
    bool show_fg_mat = false;
    bool show_track_mat = false;
    bool show_stats = false;

private:
    void _ProcessNewFrame(cv::Mat currFrame, double frameTime) override; // Run every time a new frame is available
    void _StartCalled() override;
    void _StopCalled() override;                    // Use to destroy all opencv windows
    void _imshow(std::string name, cv::Mat &image); // my own imshow to track the windows we opened
    void _imunshow(std::string name);               // deletes the window if it exists
    std::unordered_set<std::string> cvWindows;

    // Update our data
    void _UpdateData(double timestamp);
    void _UpdateOdometry(OdometryData &data, OdometryData &oldData, RobotTracker *tracker);
    void UpdateSettings(); // Bring in user settings

    Clock clock_outer; // Clock to tell us when to update statistic window (1x per second)

    bool save_video_enabled = false; // Remembers if we were saving video

    // **** Background info
    cv::Mat currBackground;       // The current background to subtract from image (cropped)
    cv::Mat currBackground_16bit; // The same background but 16-bit for averaging

    // ***** Color capable currFraem for annotating stuff
    cv::Mat currFrameColor;

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
    std::mutex _mutexAllBBoxes;
    std::vector<myRect> all_bboxes; // All the bounding boxes around foreground objects

    int fg_threshold = 20;            // Minimum intensity difference between background and fg
    double fg_threshold_ratio = 0.1f; // Minimum ratio difference of pixels

    int fg_bbox_minsize = 30;                      // minimum size of a bounding box we want to consider
    cv::Size fg_mask_blur_size = cv::Size(15, 15); // Blurring size
    int fg_post_blur_threshold = 90;
    int fg_contour_downsize = 2;     // Reduction of resolution when looking at contours
    int fg_contour_bbox_growth = 5;  // Number of pixels on each side to grow bbox to allow them to easily be combined
    int fg_max_bbox_dimension = 300; // Maximum dimension of a bbox before we reject it

    // Robot Tracking
    std::vector<RobotTracker *> _allRobotTrackers;
    double currTime = 0.0; // Current frame time
    RobotTracker *ourRobotTracker = NULL;
    RobotTracker *opponentRobotTracker = NULL;
    int maxDimension = 150;               // Maximum dimension of a bounding box
    double deleteForNoMovementTime = 0.7; // Number of s to delete a tracked object for not moving
    int deleteForNoTrackingCount = -1;    // Number of frames to delete a tracked object that hasn't locked on

    // *********  Functions
    void removeShake(cv::Mat &image);
    void healBackground(cv::Mat &currFrame);
    void ReinitBackground();
    bool SaveBackground(void); // Saves the current background to the default file

    cv::Mat regularBackground; // Recalled 8-bit grayscale background from file
    void LoadBackground(cv::Mat &currFrame);
    bool DumpBackground(std::string name, std::string path); // Will continueously dump background to files evey 1s
    void ExtractForeground(cv::Mat &croppedFrame);
    RobotTracker *AddTrackedItem(cv::Rect bbox);
    void _allRobotTrackersClear(); // clears and deletes all robot trackers
    void DrawAllBBoxes(cv::Mat &mat, int thickness = 1, cv::Scalar scaler = cv::Scalar(255, 0, 0));

    std::mutex _mutexTrackData;
    std::condition_variable_any conditionVarTrackRobots;
    int max_distance_to_locate = 200;

    void TrackRobots(cv::Mat &croppedFrame, cv::Mat &frameToDisplay);
    bool LocateRobots(cv::Point2f newPos, bool opponentRobot);
    void _deleteTracker(RobotTracker *staleTracker);

    // Video Saving for debugging
    cv::VideoWriter video;
    double time_to_stop_video = 120.0;
    void SaveToVideo(cv::Mat &image);
    void DumpRobotTrackerInfo(cv::Mat &channel, std::string title);
    void DumpCurrFrameInfo(cv::Mat &channel, cv::Mat &currForeground, std::string title);
};
