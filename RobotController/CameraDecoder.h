#pragma once
#include <opencv2/core.hpp>
#include "CameraReceiver.h"
#include "VisionPreprocessor.h"
#include "ThreadPool.h"
#include "RobotTracker.h"


// ***********************
// Camera Decoder Class
class CameraDecoder
{
public:
    CameraDecoder(ICameraReceiver &overheadCam);
    void processNewFrame(cv::Mat& newFrame);

    bool enable_camera_antishake = false;  // Turn off/on anti shake
    bool enable_background_healing = true; // Turn off/on background healing
    bool useMultithreading = false;

    // Initial Robot localization
    double time_match_starts = 2.0*21.0; // Time the match almost starts (e.g. 3/2/1)
    cv::Rect areaToLookForLeftRobot = cv::Rect(10, 220, 200, 200);
    cv::Rect areaToLookForRightRobot = cv::Rect(490, 130, 200, 200);

    // Debug
    bool save_to_video_match_debug = true; // tracking robot info dump
    bool save_to_video_output = false; 
    bool save_background_to_files = false; // enable dumping background to files
    std::string dumpBackgroundsPath = "backgrounds/dump";
    std::string loadBackgroundsPath = "backgrounds"; // Please to look for preloaded backgrounds
    std::string outputVideoFile = "Videos/matchingData.mp4";

private:
    
    
    std::thread processingThread; // Run as seperate thread to process incoming images
    ICameraReceiver& overheadCam; // The overhead camera to get raw image from

    // Vision preprocessor
    VisionPreprocessor birdsEyePreprocessor;

    // **** Background info
    cv::Mat currBackground; // The current background to subtract from image (cropped)
    cv::Mat currBackground_16bit; // The same background but 16-bit for averaging

    // ***** Color capable currFraem for annotating stuff
    cv::Mat currFrameColor;

    // Anti-Shake
    int x_offset = 5;  // x offset of camera for cropped imag. Should be >= -1*crop_x and <= crop_x
    int y_offset = 5;  // y offset of camera. Should be >= -1*crop_y and <= crop_y
    int crop_x = 5;   // number of pixels to crop on each side for x. We can scan +/- this amount.
    int crop_y = 5;   // number of pixels to crop on each side for y. We can scan +/- this amount.
    int antiShakeSize = 50; // The image size to scan for anti-shake
    int x_start_region = 300; // X Position in the background to start comparing
    int y_start_region = 5; // Y position in the background to start comparing
    int max_shake_correction = 3; // Only allow up to 3 pixel correction

    // Background healing options
    bool _enHealBackground = false; // Will be enabled once time-to-start passes
    int averagingCount = 90;  // Rolling average scaling factor for background areas
    int trackedAvgCount = 200; // Rolling average scaling factor for untracked bbox areas
    int robotAvgCount = 900; // NOT USED: Rolling average scaling factor for robot areas

    // Foreground threshold
    cv::Mat foreground; // The masked out foreground image
    cv::Mat fg_mask;    // The mask that defines the foreground
    std::vector<myRect> all_bboxes;  // All the bounding boxes around foreground objects

    int fg_threshold = 20; // Minimum intensity difference between background and fg
    int fg_bbox_minsize = 30; // minimum size of a bounding box we want to consider
    cv::Size fg_mask_blur_size = cv::Size(15,15); // Blurring size
    int fg_post_blur_threshold = 90;
    int fg_contour_downsize = 2; // Reduction of resolution when looking at contours
    int fg_contour_bbox_growth = 5; // Number of pixels on each side to grow bbox to allow them to easily be combined
    int fg_max_bbox_dimension = 300; //Maximum dimension of a bbox before we reject it

    // Robot Tracking
    bool leftRobotFound = false;
    bool rightRobotFound = false;
    double currTime = 0.0; // Time since program start at the beggining of the frame ()
    std::vector<RobotTracker*> allTrackedItems;
    RobotTracker* ourRobot = NULL;
    int maxDimension = 150; // Maximum dimension of a bounding box
    double deleteForNoMovementTime = 0.7; // Number of s to delete a tracked object for not moving
    int deleteForNoTrackingCount = -1; // Number of frames to delete a tracked object that hasn't locked on


    // *********  Functions
    void removeShake( cv::Mat& image);
    void healBackground(cv::Mat& currFrame);
    void ReinitializeBackground(cv::Mat& newBG);

    cv::Mat regularBackground; // Recalled 8-bit grayscale background from file
    void LoadBackgrounds(cv::Mat& currFrame);
    bool SaveBackground(std::string name);
    bool LocateRobots(cv::Mat& currFrame); // Tries to locate our starting bots in the confines of the areas given
    void ExtractForeground(cv::Mat& croppedFrame); 
    RobotTracker* AddTrackedItem(cv::Rect bbox);
    void DrawAllBBoxes(cv::Mat& mat, int thickness = 1, cv::Scalar scaler = cv::Scalar(255,0,0));

    std::mutex mutexTrackRobots;
    std::condition_variable_any conditionVarTrackRobots;
    void TrackRobots(cv::Mat& croppedFrame, cv::Mat& frameToDisplay);

    // Video Saving for debugging
    cv::VideoWriter  video;
    double time_to_stop_video =120.0;
    void SaveToVideo(cv::Mat& image);
    void DumpRobotTrackerInfo(cv::Mat& channel, std::string title);
    void DumpCurrFrameInfo(cv::Mat& channel, cv::Mat& currForeground, std::string title);
};
