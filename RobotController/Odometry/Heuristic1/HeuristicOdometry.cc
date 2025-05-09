#include <iostream>
#include "../../Globals.h"
#include <vector>
#include <algorithm>
#include <iterator>
#include <signal.h>
#include <windows.h>
#include <thread>
#include <sys/stat.h>
#include <filesystem>
#include <condition_variable>
#include <functional>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/flann.hpp>
#include "../../MathUtils.h"
#include "../../RobotConfig.h"
#include "HeuristicOdometry.h"
#include "../../UIWidgets/ImageWidget.h"
#include "../../RobotController.h"
#include "../../SafeDrawing.h"

// ****************************************
// CameraDecoder
// ****************************************

HeuristicOdometry::HeuristicOdometry(ICameraReceiver *videoSource) : OdometryBase(videoSource)
{

}

// Statemachine definitions
enum CAMDECODER_SM
{
    CMSM_LOADBACK = 0,
    CMSM_WAIT_TO_START,
    CMSM_TRACKINGOK
};

CAMDECODER_SM camStateMachine = CMSM_LOADBACK;

double timeTrackForBGSave = 0;
cv::Size HeuristicOdometry::bgOverlaySize = cv::Size(0,0);
cv::Mat HeuristicOdometry::backgroundOverlay = cv::Mat::zeros(0,0, CV_8UC3);


void HeuristicOdometry::UpdateSettings()
{
    averagingCount = HEU_BACKGROUND_AVGING;
    // add sanity check 
    if( averagingCount < 4) { averagingCount = 4; } 

    trackedAvgCount = HEU_UNTRACKED_MOVING_BLOB_AVGING;
    fg_threshold = HEU_FOREGROUND_THRESHOLD;
    fg_threshold_ratio = HEU_FOREGROUND_RATIO / 100.0f;
    fg_bbox_minsize = HEU_FOREGROUND_MINSIZE;
    fg_mask_blur_size = cv::Size(HEU_FOREGROUND_BLURSIZE, HEU_FOREGROUND_BLURSIZE);
    preMaskBlurSize = cv::Size(HEU_BACKGROUND_BLURSIZE, HEU_BACKGROUND_BLURSIZE);
    fg_contour_bbox_growth = HEU_FOREGROUND_BUFFER;
    blurCount = HEU_BLUR_COUNT;
    RobotTracker::moveTowardsCenter = (float)HEU_POSITION_TO_CENTER_SPEED / 100.0f;
    RobotTracker::robotVelocitySmoothing = ((float)HEU_VELOCITY_AVERAGING) / 100.0f;
    RobotTracker::useMultithreading = HEU_ROBOT_PROCESSORS > 1;
    RobotTracker::numberOfThreads = max(1, HEU_ROBOT_PROCESSORS);
}

void HeuristicOdometry::MatchStart(cv::Point2f robotPos, cv::Point2f opponentPos)
{
    // check if running, start us if not
    if (!IsRunning())
    {
        RobotController::GetInstance().odometry.Run(OdometryAlg::Heuristic);
    }

    // First reinitialize background
    match_start_bg_init = true;
    while (match_start_bg_init)
    {
        Sleep(1);
    }

    // Now lock onto robots. Set position here tries to find the best BBox closes to the position
    // It does not actually set it to that position
    SetPosition(robotPos, false);
    SetPosition(opponentPos, true);
}


// Called in CameraDecoder thread to process the new frame
void HeuristicOdometry::_ProcessNewFrame(cv::Mat currFrame, double frameTime)
{
    

    timing_list.clear();
    timing_text.clear();
    timing_clock.markStart();

    currTime = frameTime;

    markTime("Start: ");

    UpdateSettings(); // Update any settings from user

    // Change the picute to 8-bit B&W
    cv::Mat newFrame;

    // Convert to greyscale if required
    if (currFrame.channels() == 1)
    {
        newFrame = currFrame;
    }
    else if (currFrame.channels() == 3)
    {
        cv::cvtColor(currFrame, newFrame, cv::COLOR_BGR2GRAY);
    }
    else if (currFrame.channels() == 4)
    {
        cv::cvtColor(currFrame, newFrame, cv::COLOR_BGRA2GRAY);
    }

    // Crop the image (not required if anti-shake not used)
    cv::Rect cropparea(x_offset, y_offset, newFrame.cols - 2 * crop_x, newFrame.rows - 2 * crop_y);
    cv::Mat croppedFrame = newFrame(cropparea);

    bool matchstart_was_run = false;

    if( match_start_bg_init)
    {
        // First override all variables that matchstatbackground algorithm uses different
        // OVerride fg_threshold, blurr size nad blur count
        fg_threshold = HEU_FOREGROUND_THRESHOLD_INIT;
        blurCount = HEU_BLUR_COUNT_INIT;
        preMaskBlurSize = cv::Size(HEU_BACKGROUND_BLURSIZE_INIT, HEU_BACKGROUND_BLURSIZE_INIT);
  
        MatchStartBackgroundInit(croppedFrame);
        
        matchstart_was_run = true;
    }
    else
    {
        // Correct Brightness (assumes shake doesn't significantly affect it on average)
        correctBrightness(croppedFrame);
    }

    // Find the proper x_offset and y_offset for a shaky image using background
    // calcShakeRemoval(newFrame);

    markTime("Brightness and shake: ");

//  _imshow("corrected", newFrame);



    // ***********************************************
    // ******* SERVICE USER REQUESTS *****************

    if (save_background)
    {
        SaveBackground();
        save_background = false;
    }

    if (reinit_bg)
    {
        ReinitBackground();
        reinit_bg = false;
    }

    if( set_currFrame_to_bg)
    {
        set_currFrame_to_bg = false;
        croppedFrame.copyTo(regularBackground);
        ReinitBackground();
    }



 

    if (save_to_video_match_debug || save_to_video_output) // Remeber that we are saving video
    {
        save_video_enabled = true;
    }
    else if (save_video_enabled) // If no longer, release video
    {
        save_video_enabled = false;
        video.release();
    }

    // Save background to file
    if (save_background_to_files && (currTime - timeTrackForBGSave > 1.0))
    {
        DumpBackground("background_" + std::to_string((int)currTime), dumpBackgroundsPath);
        timeTrackForBGSave = currTime;
    }

    // ##############################
    // Begin access of core tracking data lists

    // From here on maniuplation of all_bboxes and/or tracker (core tracking data) is allowed
    std::unique_lock<std::mutex> locker(_mutexAllBBoxes);

    if (load_background)
    {
        LoadBackground(newFrame, loadBackgroundsPath + "/savedBackground.jpg"); // Use savebackground path
        load_background = false;
    }

    if (load_start_background)
    {
        LoadBackground(newFrame); // Loads image start background by default
        load_start_background = false;
    }

    // ************************************
    // FOREGROUND and BBOX EXTRACTION

    // Extract foreground
    ExtractForeground(croppedFrame);

    // Create a color version of the frame to display stats on
    cv::cvtColor(croppedFrame, currFrameColor, cv::COLOR_GRAY2BGR);

    markTime("Extract Foreground: ");

    // *************************************************
    // ROBOT/ITEM Tracking

    cv::Mat frameToDisplay = newFrame;

    // Lock all RobotTrackers from being access
    TrackRobots(croppedFrame, currFrameColor);

    // Heal background
    healBackground(croppedFrame);

    markTime("Heal bg: ");

    // static ImageWidget background{"Background", currBackground, false};
    if (show_bg_mat)
    {
        bgOverlaySize = currBackground.size();

        // See if there needs to be a merged image
        if(  !backgroundOverlay.empty() && (backgroundOverlay.size() == currBackground.size() ))
        {
            try{
                cv::Mat resultimage = backgroundOverlay.clone();
                cv::Mat convertedBackground;
                cv::cvtColor( currBackground, convertedBackground,  cv::COLOR_GRAY2BGR);
                cv::addWeighted(convertedBackground,1, resultimage, 1, 0, resultimage);
                _imshow("background", resultimage); 
            }
            catch(...)
            {
                _imshow("background", currBackground); 
            }
        }
        else
        {
            _imshow("background", currBackground); 
        }

    }
    else
    {
        _imunshow("background");
        bgOverlaySize = cv::Size(0,0);
    };

    DrawAllBBoxes(currFrameColor);

    if( matchstart_was_run)
    {        

        if( HEU_HEAL_BG_INIT)
        {
            recreateBackgroundOnMatchStart(croppedFrame);
        }
     
        cv::Mat cleanedup = foreground.clone();

        DumpCurrFrameInfo(cleanedup, foreground, "cleanup_fg");
     
        // Return settings back to normal
        UpdateSettings();
        match_start_bg_init = false;
    }

    // End of access to core tracking lists
    // ###################################

    // ********************
    // Update our data
    _UpdateData(frameTime);
    locker.unlock();

    // Add the debug string
    if( show_track_mat || save_video_enabled && save_to_video_output)
    {
        std::lock_guard<std::mutex> lock(debugROStringForVideo_mutex);
        AddDebugStringToFrame(currFrameColor, debugROStringForVideo);
    }

    // Add foreground onto the color frame
    cv::Mat fg_color;
    cv::cvtColor(fg_mask, fg_color, cv::COLOR_GRAY2BGR);
    fg_color.setTo(cv::Scalar(255, 100, 100), fg_mask);

    cv::addWeighted(currFrameColor, 1.0f, fg_color, 0.3f, 0, currFrameColor);

    // Finaly add the actual robot tracking circles
    OdometryData robotData = RobotController::GetInstance().odometry.Robot();
    OdometryData opponentData = RobotController::GetInstance().odometry.Opponent();

    if( robotData.robotPosValid)
    {
        safe_circle(currFrameColor, robotData.robotPosition, 10, cv::Scalar(0, 255, 100), 2, cv::LINE_AA, 0);
    }

    if( opponentData.robotPosValid)
    {
        safe_circle(currFrameColor, opponentData.robotPosition, 10, cv::Scalar(0, 100, 255), 2, cv::LINE_AA, 0);
    }


    currFrameColor.copyTo(prevFrameColor);

    if (show_track_mat)
    {
        _imshow("HeuristicData", currFrameColor);
    }
    else
    {
        _imunshow("HeuristicData");
    };

    if (show_fg_mat)
    {
        _imshow("Foreground", foreground);
    }
    else
    {
        _imunshow("Foreground");
    };

    // Service and requests from user (this needs to be done here)
    if (save_video_enabled && save_to_video_output)
    {
        SaveToVideo(currFrameColor);
    }

    // Write timing information to my window
    if (clock_outer.getElapsedTime() > 1.0)
    {
        cv::Mat statsmat = cv::Mat::zeros(800, 800, CV_8UC3);
        int y_offset = 20;

        for (int i = 0; i < timing_list.size(); i++)
        {
            printText(timing_text[i] + std::to_string(timing_list[i] * 1000.0) + "ms", statsmat, y_offset);
            y_offset += 20;
        }

        if (show_stats)
        {
            _imshow("Statistics", statsmat);
        }
        else
        {
            _imunshow("Statistics");
        };

        clock_outer.markStart();
    }


    // This updates images
    cv::pollKey();
}

// Prints the debugString to the frame
void HeuristicOdometry::AddDebugStringToFrame(cv::Mat &frame, std::string debugString)
{
    int y_offset = 20;
    std::vector<std::string> lines;
    std::istringstream f(debugString);
    std::string line;
    while (std::getline(f, line))
    {
        lines.push_back(line);
    }

    for (int i = 0; i < lines.size(); i++)
    {
        printText(lines[i], frame, y_offset);
        y_offset += 20;
    }
}

void HeuristicOdometry::_UpdateData(double timestamp)
{
    // Get unique access
    std::unique_lock<std::mutex> locker(_updateMutex);

    // Make a copy of currData for velocity calls
    OdometryData _prevDataRobot = _currDataRobot;
    OdometryData _prevDataOpponent = _currDataOpponent;

    _currDataRobot.id++;                // Increment frame id
    _currDataRobot.frameID = frameID; // Set to new frame id
    _currDataRobot.time = timestamp;    // Set to new time
    _currDataRobot.time_angle = timestamp;    // Set to new time
    _currDataOpponent.id++;             // Increment frame id
    _currDataOpponent.frameID = frameID; // Set to new frame id
    _currDataOpponent.time = timestamp; // Set to new time
    _currDataOpponent.time_angle = timestamp; // Set to new time

    // Clear curr data
    _currDataRobot.Clear();
    _currDataRobot.isUs = true; // Make sure this is set
    _currDataOpponent.Clear();
    _currDataOpponent.isUs = false; // Make sure this is set

    // Update our robot position/velocity/angle
    _UpdateOdometry(_currDataRobot, _prevDataRobot, ourRobotTracker);
    _UpdateOdometry(_currDataOpponent, _prevDataOpponent, opponentRobotTracker);
}

void HeuristicOdometry::_UpdateOdometry(OdometryData &data, OdometryData &oldData, RobotTracker *tracker)
{
    if (tracker == nullptr)
    {
        // If no tracker found, keep previous position (no velocities)
        data.robotPosition = oldData.robotPosition;
        data.robotAngle = oldData.robotAngle;
        data.robotVelocity = cv::Point2f(0,0);
        data.robotAngleVelocity = 0;
        data.robotPosValid = false;
        data.robotAngleValid = false;
        return;
    }

    data.robotPosValid = true;
    data.robotPosition = tracker->position.Point2f();
    data.robotVelocity = tracker->avgVelocity.Point2f();
    data.rect = tracker->bbox;

    data.robotAngleValid = true;
    data.robotAngle = Angle(tracker->rotation.angleRad());

    data.robotAngleVelocity = 0;

    double deltaTime = data.time - oldData.time;
    if ((deltaTime > 0) && (deltaTime < angleVelocityTimeConstant) )
    {
        data.robotAngleVelocity = data.robotAngleVelocity * (1.0 - deltaTime / angleVelocityTimeConstant) + angleVelocityTimeConstant * (data.robotAngle - oldData.robotAngle);
    }
}

void HeuristicOdometry::SetPosition(cv::Point2f newPos, bool opponentRobot)
{
    // First lock for all robot tracking and find the robot
    std::unique_lock<std::mutex> locktracking(_mutexAllBBoxes);
    LocateRobots(newPos, opponentRobot);

    locktracking.unlock();

    // Now update the data
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;
    RobotTracker *tracker = (opponentRobot) ? opponentRobotTracker : ourRobotTracker;

    odoData.robotPosValid = tracker != nullptr;
    odoData.robotPosition = (odoData.robotPosValid) ? tracker->position.Point2f() : newPos;
    odoData.robotVelocity = cv::Point2f(0, 0);
    odoData.time = Clock::programClock.getElapsedTime();
}

void HeuristicOdometry::ForcePosition(cv::Point2f newPos, bool opponentRobot)
{
    // Here we assume newPos is inside the current tracked box.
    // If we have no tracker, use setPosition First
    // If setPosition fails, than continue on with force position
    std::unique_lock<std::mutex> locktracking(_mutexAllBBoxes, std::defer_lock);

    if (opponentRobot && opponentRobotTracker == nullptr)
    {
        // It will lock mallboxes
        SetPosition(newPos, opponentRobot);

        // Now we need to lock it ourselves
        locktracking.lock();
        if (opponentRobotTracker == nullptr)
        {
            _currDataOpponent.robotPosValid = false;
            return;
        }
    }
    else if (!opponentRobot && ourRobotTracker == nullptr)
    {
        // It will lock mallboxes
        SetPosition(newPos, opponentRobot);

        // Now we need to lock it ourselves
        locktracking.lock();
        if (ourRobotTracker == nullptr)
        {
            _currDataRobot.robotPosValid = false;
            return;
        }
    }
    else
    {
        locktracking.lock();
    }

    RobotTracker *&firstbot = (opponentRobot) ? opponentRobotTracker : ourRobotTracker; // Guaranteed not to be nullpts
    RobotTracker *&secondbot = (opponentRobot) ? ourRobotTracker : opponentRobotTracker; // could be nullptr
    
    // Next we have have the following scenario:
    // 1) newPos is inside our robot and our tracker is not combined
    // 2) newPos is insside our robot and it is a combined tracker
    //  -> Do the same thing, just set the position
    // 3) New position is inside opponent robot and our robot is not combined
    //   ->Swap positions and set position on our new swapped foreground
    // 4) Position is outside all of us
    //   -> Locate robots and force position

    int scenario = 0;

    // Case 1/2
    if(firstbot->IsPointInside( newPos) && !firstbot->IsTrackerCombined())
    {
        scenario = 1;
    }
    else if(firstbot->IsPointInside( newPos) && firstbot->IsTrackerCombined())
    {
        scenario = 2;
    }
    else if( secondbot != nullptr && secondbot->IsPointInside( newPos) )
    {
        scenario = 3;
    }
    else
    {
        scenario = 4;
    }

    switch( scenario)
    {
        case 1:
        case 2:
                firstbot->position = newPos;                
                break;
        case 3:
                secondbot->position = newPos;  
                // Will need to swap robots later            
                break;
        case 4:
                LocateRobots(newPos, opponentRobot);
                if( firstbot != nullptr)
                {
                    firstbot->position = newPos;  
                }
                break;
    }

    locktracking.unlock();


    // If this was a case 3, swap robots
    if( scenario == 3)
    {
        SwitchRobots();
    }

    // Force the position
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;

    odoData.robotPosition = newPos;
    odoData.robotPosValid = true;
}

void HeuristicOdometry::SetAngle(double newAngle, bool opponentRobot)
{
    // First lock for all robot tracking and find the robot
    std::unique_lock<std::mutex> locktracking(_mutexAllBBoxes);

    // Update robot trackers
    RobotTracker *tracker = (opponentRobot) ? opponentRobotTracker : ourRobotTracker;

    if (tracker != NULL)
    {
        tracker->SetRotation(newAngle);
    }
    locktracking.unlock();

    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;

    odoData.robotAngle = Angle(newAngle);
    odoData.robotAngleValid = true;
    odoData.robotAngleVelocity = 0;
}

void HeuristicOdometry::SetVelocity(cv::Point2f newVel, bool opponentRobot)
{
    // First lock for all robot tracking and find the robot
    std::unique_lock<std::mutex> locktracking(_mutexAllBBoxes);

    // Update robot trackers
    RobotTracker *tracker = (opponentRobot) ? opponentRobotTracker : ourRobotTracker;

    if (tracker != NULL)
    {
        tracker->avgVelocity.x = newVel.x;
        tracker->avgVelocity.y = newVel.y;
    }
    locktracking.unlock();

    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;

    odoData.robotVelocity = newVel;
}

void HeuristicOdometry::SetAngularVelocity(double newVel, bool opponentRobot)
{
    // First lock for all robot tracking and find the robot
    std::unique_lock<std::mutex> locktracking(_mutexAllBBoxes);

    // Update robot trackers
    RobotTracker *tracker = (opponentRobot) ? opponentRobotTracker : ourRobotTracker;

    if (tracker != NULL)
    {
        // tracker->SetRotation(newAngle);
    }
    locktracking.unlock();

    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;

    odoData.robotAngleVelocity = newVel;
}


void HeuristicOdometry::SwitchRobots(void)
{
    // First lock for all robot tracking and find the robot
    std::unique_lock<std::mutex> locktracking(_mutexAllBBoxes);
    RobotTracker *tempTracker = ourRobotTracker;
    ourRobotTracker = opponentRobotTracker;
    opponentRobotTracker = tempTracker;
    locktracking.unlock();

    // Switch who's who
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData temp_Robot = _currDataRobot;
    _currDataRobot = _currDataOpponent;
    _currDataOpponent = temp_Robot;

    _currDataRobot.isUs = true;
    _currDataOpponent.isUs = false;
}

void HeuristicOdometry::_imshow(std::string name, cv::Mat &image)
{
    cv::imshow(name, image);
    cvWindows.insert(name);
}

void HeuristicOdometry::_imunshow(std::string name)
{
    if (cvWindows.erase(name) != 0)
    {
        cv::destroyWindow(name);
    }
}

void HeuristicOdometry::_StartCalled()
{
    cvWindows.clear();
}

void HeuristicOdometry::_StopCalled()
{
    // destroy all the cv windows
    for (const auto &name : cvWindows)
    {
        cv::destroyWindow(name);
    }
}

void HeuristicOdometry::TrackRobots(cv::Mat &croppedFrame, cv::Mat &frameToDisplay)
{
    int starting_y = 0;

    // *** DEBUG video dump *****
    cv::Mat nullMat;
    cv::Mat videoFrame;
    std::vector<cv::Mat> channels;
    if (save_video_enabled && save_to_video_match_debug)
    {
        videoFrame = cv::Mat::zeros(frameToDisplay.size(), frameToDisplay.type());
        cv::split(videoFrame, channels);

        // Record starting info
        DumpRobotTrackerInfo(channels[2], "Start Data");

        // Record currFrame data
        DumpCurrFrameInfo(channels[1], foreground, "           Current Data");
    }
    // *************************************************
    // ROBOT/ITEM Tracking

    // Step 1: Let every robot mark which FG Box is the best fit
    //
    // Step 2: Let all the tracked items process the bboxes. If the bbox is asked from multiple robots, then treat it like overlly big boxes

    // Step 1:  Let every robot mark which FG Box is the best fit
    for (auto currItem : _allRobotTrackers)
    {
        currItem->FindBestBBox(currTime, all_bboxes);
    }

    // Step 2:
    // Go through each tracker and and process current frame
    int processes_run = 0;
    int processes_done = 0;
    int i = 0;
    int itarget = 0; // The number of the tracked item to output
    for (auto currItem : _allRobotTrackers)
    {
        markTime("Starting Tracked Item # " + std::to_string(processes_run) + ": ");
        if (!useMultithreading)
        {
            currItem->ProcessNewFrame(currTime, foreground, croppedFrame, fg_mask, processes_done, conditionVarTrackRobots, _mutexTrackData, (save_video_enabled && save_to_video_match_debug && i == itarget) ? channels[1] : nullMat);
        }
        else
        {
            ++processes_run;
            auto boundFunction = std::bind(&RobotTracker::ProcessNewFrame, currItem, currTime, std::ref(foreground),
                                           std::ref(croppedFrame), std::ref(fg_mask), std::ref(processes_done), std::ref(conditionVarTrackRobots), std::ref(_mutexTrackData), std::ref((save_video_enabled && save_to_video_match_debug && i == itarget) ? channels[1] : nullMat));

            ThreadPool::myThreads.enqueue(boundFunction);
        }
        i++;
    }

    markTime("Track Items all started : ");

    _mutexTrackData.lock();
    while (processes_done < processes_run)
    {
        if (conditionVarTrackRobots.wait_for(_mutexTrackData, std::chrono::seconds(2)) == std::cv_status::timeout)
        {
            markTime("Timeout occured in CameraDecoder::TrackRobots!! ");
        }
    }
    _mutexTrackData.unlock();

    markTime("Track Items all joined : ");

    if (save_video_enabled && save_to_video_match_debug)
    {
        DumpRobotTrackerInfo(channels[0], "                           End Data");
    }

    // Draw the rectangle around our tracked items
    for (auto currIter = _allRobotTrackers.begin(); currIter != _allRobotTrackers.end();)
    {
        // Delete items that haven't been tracked for a while
        if ((deleteForNoTrackingCount > 0) && ((*currIter)->numFramesNotTracked > deleteForNoTrackingCount))
        {
            _deleteTracker(*currIter);
            currIter = _allRobotTrackers.erase(currIter);
        }
        else
        {
            // Draw the rectangle of this tracked bot
            // Draw a Green circle around us
            cv::Scalar colorRectangle =  cv::Scalar(0, 255, 255);

            if ((*currIter) == ourRobotTracker)
            {
                colorRectangle = cv::Scalar(0, 255, 0);
            }
            else if ((*currIter) == opponentRobotTracker)
            {
                colorRectangle = cv::Scalar(0, 0, 255);
            }
            cv::rectangle(frameToDisplay, (*currIter)->bbox, colorRectangle, 3);
            safe_circle(frameToDisplay, (*currIter)->GetCenter(), 10, colorRectangle, 3);
            cv::Point rotationdir((*currIter)->rotation.x * 50.0, (*currIter)->rotation.y * 50.0);
            cv::Point centerBBox = (*currIter)->bbox.tl();
            centerBBox.x += (*currIter)->bbox.width / 2;
            centerBBox.y += (*currIter)->bbox.height / 2;

            cv::Point avgVelocityPoint((*currIter)->avgVelocity.x / 3.0, (*currIter)->avgVelocity.y / 3.0);

            // Draw avg velocity
            //  cv::line(frameToDisplay, centerBBox, centerBBox + avgVelocityPoint, cv::Scalar(0,10,150), 3 );

            // Draw rotation
            cv::line(frameToDisplay, centerBBox, centerBBox + rotationdir, cv::Scalar(0, 50, 250), 2);

            int line = 0;
            printText("Bbox w,h=" + std::to_string((*currIter)->bbox.width) + "," + std::to_string((*currIter)->bbox.height),
                      frameToDisplay, (*currIter)->bbox.y + 20 * line++, (*currIter)->bbox.x + (*currIter)->bbox.width);
            printText("Velocity=" + std::to_string((*currIter)->avgVelocity.mag()),
                      frameToDisplay, (*currIter)->bbox.y + 20 * line++, (*currIter)->bbox.x + (*currIter)->bbox.width);
            printText("velForMove=" + std::to_string((*currIter)->velForMovementDetection.mag()),
                      frameToDisplay, (*currIter)->bbox.y + 20 * line++, (*currIter)->bbox.x + (*currIter)->bbox.width);
            // printText("noMoving=" + std::to_string((*currIter)->timeNotMoving),
            //     frameToDisplay, (*currIter)->bbox.y+20*line++, (*currIter)->bbox.x+ (*currIter)->bbox.width);
            // printText("noTracking=" + std::to_string((*currIter)->numFramesNotTracked),
            //     frameToDisplay, (*currIter)->bbox.y+20*line++, (*currIter)->bbox.x+ (*currIter)->bbox.width);
            printText("delta=(" + std::to_string((*currIter)->delta.x) + "," + std::to_string((*currIter)->delta.y) + ")",
                      frameToDisplay, (*currIter)->bbox.y + 20 * line++, (*currIter)->bbox.x + (*currIter)->bbox.width);
            double angle = rad2deg(atan2((*currIter)->rotation.y, (*currIter)->rotation.x));
            printText("Rotation=" + std::to_string(angle),
                      frameToDisplay, (*currIter)->bbox.y + 20 * line++, (*currIter)->bbox.x + (*currIter)->bbox.width);

            ++currIter;
        }
    }

    // Merge the channels
    if (save_video_enabled && save_to_video_match_debug)
    {
        cv::merge(channels, videoFrame);

        SaveToVideo(videoFrame);
    }
}

void HeuristicOdometry::_deleteTracker(RobotTracker *staleTracker)
{
    if (staleTracker == nullptr)
    {
        return;
    }

    if (ourRobotTracker == staleTracker)
    {
        ourRobotTracker = nullptr;
    }
    if (opponentRobotTracker == staleTracker)
    {
        opponentRobotTracker = nullptr;
    }
    delete (staleTracker);
}

// Adds the foreground around the bbox specified to the tracked list
RobotTracker *HeuristicOdometry::AddTrackedItem(cv::Rect bbox)
{
    if( bbox.x > 720 || bbox.y > 720)
    {
        printf("Invalid bbox passed to AddTrackedItem\n");
    }

    RobotTracker *newItem = new RobotTracker(bbox);
    _allRobotTrackers.push_back(newItem);

    // Initialize items data
    newItem->fg_image = foreground(bbox).clone();
    newItem->fg_mask = fg_mask(bbox).clone();
    newItem->lastTime = currTime;

    return newItem;
}

void HeuristicOdometry::_allRobotTrackersClear()
{
    for (auto currIter = _allRobotTrackers.begin(); currIter != _allRobotTrackers.end();)
    {
        _deleteTracker(*currIter);
    }

    _allRobotTrackers.clear();
}

// Saves currBackground to a file
bool HeuristicOdometry::SaveBackground()
{
    currBackground.copyTo(regularBackground);
    return DumpBackground("savedBackground", loadBackgroundsPath);
}

void HeuristicOdometry::LoadBackground(cv::Mat &currFrame, std::string image_name)
{
    // Center anti-shake cropping
    x_offset = crop_x;
    y_offset = crop_y;

    // First load the regular Background. This should be an 8-bit image ideally
    // This should also be cropped already
    if( image_name.length() < 1)
    {
        image_name = IMAGE_START_BACKGROUND;
    }

    regularBackground = cv::imread(image_name, cv::IMREAD_GRAYSCALE);

    // Also load the reference intesnity image
    refBackground = cv::imread(IMAGE_REF_INTENSITY, cv::IMREAD_GRAYSCALE);

    // Reset the reference background average intensity calculations
    for( int i =0; i < 4; i++)
    {
        refIntensities[i] = -1.0;
    }

    // If unable to load, use current frame to set the background
    // or if background doesn't meet our dimension requirements
    if (regularBackground.empty() ||
        regularBackground.cols + 2 * crop_x != currFrame.cols ||
        regularBackground.rows + 2 * crop_y != currFrame.rows)
    {
        currFrame.copyTo(regularBackground);

        cv::Rect cropparea(crop_x, crop_y, regularBackground.cols - 2 * crop_x, regularBackground.rows - 2 * crop_y);
        regularBackground(cropparea).copyTo(regularBackground);
    }

    // Make sure the image is 8-bit
    if (regularBackground.depth() != CV_8U)
    {
        regularBackground.convertTo(regularBackground, CV_8U);
    }

    // Also correct reference background
    if (!refBackground.empty() && refBackground.depth() != CV_8U)
    {
        refBackground.convertTo(refBackground, CV_8U);
    }

    // Save this background as current background
    regularBackground.copyTo(currBackground);
    currBackground.convertTo(currBackground_16bit, CV_16U, 256);
}

// Saves currBackground to a file
bool HeuristicOdometry::DumpBackground(std::string name, std::string path)
{
    try
    {
        std::filesystem::path dirPath = path;

        if (!std::filesystem::exists(dirPath))
        {
            std::filesystem::create_directory(dirPath);
        }

        // Output the background
        int jpegQuality = 99;

        // Set the compression parameters
        std::vector<int> compressionParams;
        compressionParams.push_back(cv::IMWRITE_JPEG_QUALITY);
        compressionParams.push_back(jpegQuality);

        // Write the image to a file with the specified quality
        return cv::imwrite(dirPath.string() + "/" + name + ".jpg", currBackground, compressionParams);
    }
    catch (const std::exception &e)
    {
        return false;
    }
}

void HeuristicOdometry::SaveToVideo(cv::Mat &image)
{
    if (!save_video_enabled)
    {
        return;
    }

    // Make sure we are initialized
    if (!video.isOpened())
    {
        // Open the video
        // video = cv::VideoWriter(outputVideoFile, cv::VideoWriter::fourcc('M','J','P','G'), 60, cv::Size(image.cols,image.rows));
        video = cv::VideoWriter(outputVideoFile, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 60, cv::Size(image.cols, image.rows));

        // Failed to open, must have writing permissions wrong
        if (!video.isOpened())
        {
            return;
        }
    }

    video.write(image);
}

void HeuristicOdometry::DumpRobotTrackerInfo(cv::Mat &channel, std::string title)
{
    printText("v1.2", channel, 20);
    printText(title, channel, 60);
    int offset = -250;
    int robotid = 0;

    // Go through all the robots and add in their info
    for (auto currRobot : _allRobotTrackers)
    {
        cv::Rect bbox = currRobot->bbox;

        // Drop if x or y are outside
        if ((bbox.x < 0) || (bbox.x > channel.rows - 1) || (bbox.y < 0) || (bbox.y > channel.cols - 1))
        {
            continue;
        }

        // Correct box size
        bbox.width -= (bbox.x + bbox.width >= channel.rows) ? bbox.x + bbox.width - channel.rows : 0;
        bbox.height -= (bbox.y + bbox.height >= channel.cols) ? bbox.y + bbox.height - channel.cols : 0;

        // Drop if area is 0
        // if( bbox.area() <= 1) { continue;}

        // Add bounding box
        cv::rectangle(channel, bbox, cv::Scalar(255), 2, 4);

        // Add MAtching box
        cv::rectangle(channel, currRobot->finalMatchingROI, cv::Scalar(180), 2);

        // Add image
        // Since bbox changed, make sure image is the same width
        if (currRobot->fg_image.cols != bbox.width)
        {
            continue;
        }
        if (currRobot->fg_image.rows != bbox.height)
        {
            continue;
        }

        cv::bitwise_or(currRobot->fg_image, channel(bbox), channel(bbox));

        // Add Center Circle
        safe_circle(channel, currRobot->GetCenter(), 5, cv::Scalar(255 / 2), 2);

        // Add info
        int i = bbox.y / 20;
        int x_pos = (bbox.x + bbox.width) + 5 + offset;
        if (x_pos > channel.cols - 150)
        {
            x_pos -= 150;
        }

        if (x_pos < 0)
        {
            x_pos += 150;
        }

        printText("Loc =" + std::to_string(currRobot->position.x) + "," + std::to_string(currRobot->position.y), channel, 20 * i++, x_pos);
        printText("Rot =" + std::to_string(currRobot->rotation.angle()), channel, 20 * i++, x_pos);
        printText("Conf =" + std::to_string(currRobot->delta_confidence), channel, 20 * i++, x_pos);
        printText("Delta Loc = (" + std::to_string(currRobot->delta.x) + "," + std::to_string(currRobot->delta.y) + ")", channel, 20 * i++, x_pos);
        printText("Delta Rot =" + std::to_string(currRobot->delta_angle), channel, 20 * i++, x_pos);
        printText(currRobot->debugLine, channel, 20 * i++, x_pos);

        offset += 250;

        // Add match Result
        if (!currRobot->matchResultSaved.empty())
        {
            cv::Rect rectToOutput(cv::Point2i(robotid * 200, 0), currRobot->matchResultSaved.size());

            cv::Mat outputMatches;
            cv::normalize(currRobot->matchResultSaved, outputMatches, 0, 1.0, cv::NORM_MINMAX);

            // If our confidence is < 1, scale it
            if (currRobot->delta_confidence < 1.0)
            {
                outputMatches = outputMatches * currRobot->delta_confidence;
            }

            outputMatches.convertTo(outputMatches, CV_8U, 255.0);
            outputMatches.copyTo(channel(rectToOutput));

            if (currRobot->finalCorrectionROI.area() > 0)
            {
                cv::Mat deratingMat;
                currRobot->distanceDerating(currRobot->finalCorrectionROI).copyTo(deratingMat);

                deratingMat.convertTo(deratingMat, CV_8U, 255.0);
                cv::Rect rect2ToOutput(cv::Point2i(robotid * 200, 200), deratingMat.size());
                deratingMat.copyTo(channel(rect2ToOutput));
            }
        }

        robotid++;
    }
}

void HeuristicOdometry::DumpCurrFrameInfo(cv::Mat &channel, cv::Mat &currForeground, std::string title)
{
    printText(title, channel, 60);

    // Add timestamp
    printText("Time = " + std::to_string(currTime), channel, 80);

    // Draw all the bboxes
    DrawAllBBoxes(channel, 2, cv::Scalar(150));

    // Now copy in foreground
    cv::bitwise_or(channel, currForeground, channel);
}

// Returns true if any of the robot was found
bool HeuristicOdometry::LocateRobots(cv::Point2f newPos, bool opponentRobot)
{

    // First remove the old tracking info
    RobotTracker *&robotTracker = (opponentRobot) ? opponentRobotTracker : ourRobotTracker;

    if (robotTracker != nullptr)
    {
        // Add its box back into all_bboxes
        if(robotTracker->numFramesNotTracked < 1)
        {
            all_bboxes.push_back(robotTracker->bbox);
        }
        _allRobotTrackers.erase(std::remove(_allRobotTrackers.begin(), _allRobotTrackers.end(), robotTracker), _allRobotTrackers.end());
        _deleteTracker(robotTracker);
    }
    robotTracker = nullptr;

    // Find the closest bounding box
    cv::Rect bestBBox(0, 0, 0, 0);
    int index1 = -1;
    float distance = FindClosestBBox(all_bboxes, newPos, bestBBox, index1);

    int maxDimension = (opponentRobot) ? MAX_OPPONENT_BLOB_SIZE : MAX_ROBOT_BLOB_SIZE;
    // If we found a bbox of sufficient size that is reasonably close
    if ((index1 >= 0) && (distance < max_distance_to_locate) && (bestBBox.width >= fg_bbox_minsize) && (bestBBox.width <= maxDimension) && (bestBBox.height >= fg_bbox_minsize) && (bestBBox.height <= maxDimension))
    {
        // This is a valid bbox and add it in
        robotTracker = AddTrackedItem(bestBBox);

        // Remove it from considered boxes
        all_bboxes.erase(all_bboxes.begin() + index1);

        return true;
    }

    return false;
}

void HeuristicOdometry::DrawAllBBoxes(cv::Mat &mat, int thickness, cv::Scalar scaler)
{
    for (const auto &bbox : all_bboxes)
    {
        cv::rectangle(mat, bbox, scaler, thickness, cv::LINE_4);
        printText(std::to_string(bbox.width) + "x" + std::to_string(bbox.height), mat, bbox.y, bbox.x);
    }
}

void HeuristicOdometry::ExtractForeground(cv::Mat &croppedFrame)
{
    // ************************************
    // FOREGROUND and BBOX EXTRACTION

    // First get mask for foreground extraction
    cv::Mat currBackgroundBlurred;
    cv::Mat croppedFrameBlurred;

    currBackgroundBlurred = currBackground.clone();
    croppedFrameBlurred = croppedFrame.clone();

    // blur
    for( int i = 0; i < blurCount; i++)
    {
        cv::blur(currBackgroundBlurred, currBackgroundBlurred, preMaskBlurSize);
        cv::blur(croppedFrameBlurred, croppedFrameBlurred, preMaskBlurSize);
    }

    // First get the absolute difference (leaving >0 values where there is a difference)
    cv::Mat foregroundBlurred;
    cv::absdiff(croppedFrameBlurred, currBackgroundBlurred, foregroundBlurred);

//cv::imshow("Frame Blurred", croppedFrameBlurred);
//cv::imshow("BG Blurred", currBackgroundBlurred);
//cv::imshow("Blurred Diff", foregroundBlurred);
    // First get the mask that includes regions at least the minimum pixels away
    cv::threshold(foregroundBlurred, fg_mask, fg_threshold, 255, cv::THRESH_BINARY);

    cv::Mat diffThreshold = fg_threshold_ratio * currBackgroundBlurred;
    cv::Mat fg_maskRatio = foregroundBlurred > diffThreshold;
    cv::bitwise_and(fg_maskRatio, fg_mask, fg_mask);
//cv::imshow("Blurred Mask", fg_mask);
    // Next we want to look at pixels that changed at least a  certain percentage

    // Find contours in the mask
    // But the number it finds is way to large, thus we want to get rid of speckles
    // and only look at bigger contours
    // Do do this, lets blur the image, re-threshold, down-sample, use that for bbox discovery
    // then lets up-sample it

    // First lets blur the mask. This double blur seems to work well (balances how much backround to include versus filling in empty spots)
    cv::Mat fg_mask_blurred;
    cv::blur(fg_mask, fg_mask_blurred, fg_mask_blur_size);
    cv::threshold(fg_mask_blurred, fg_mask, fg_post_blur_threshold, 255, cv::THRESH_BINARY);
    // cv::blur(fg_mask, fg_mask_blurred, fg_mask_blur_size);
    // cv::threshold(fg_mask_blurred, fg_mask, fg_post_blur_threshold, 255, cv::THRESH_BINARY);

    // Now down-sample the image for faster countour detection
    cv::Mat fg_mask_small;
    cv::resize(fg_mask, fg_mask_small, fg_mask.size() / fg_contour_downsize);

    std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(fg_mask_small, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // cv::findContours(fg_mask_small, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);
    cv::findContours(fg_mask_small, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_KCOS); // Seems to have less points then Simple

    // Lets fill all contours
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::drawContours(fg_mask_small, contours, (int)i, cv::Scalar(255), cv::FILLED);
    }

    // Coppy over the filled contours to the main FG mask
    cv::Mat regrown_mask;
    cv::resize(fg_mask_small, regrown_mask, fg_mask.size());
    cv::bitwise_or(fg_mask, regrown_mask, fg_mask);

    // Find the bounding box for each contour
    all_bboxes.clear();
    for (const auto &contour : contours)
    {

        myRect bbox = cv::boundingRect(contour);
        bbox.x *= fg_contour_downsize;
        bbox.y *= fg_contour_downsize;
        bbox.width *= fg_contour_downsize;
        bbox.height *= fg_contour_downsize;

        // Grow the bounding box a little so they can combine if very close together
        // At this point they could reach beyond the screen boundries
        bbox.x -= fg_contour_bbox_growth;
        bbox.y -= fg_contour_bbox_growth;
        bbox.width += 2 * fg_contour_bbox_growth;
        bbox.height += 2 * fg_contour_bbox_growth;
        all_bboxes.push_back(bbox);
    }

    // Combine overlapping bounding boxes
    double num_of_operations = 0;
    std::vector<bool> erased_bbox(all_bboxes.size(), false);

    bool combined;
    do
    {
        combined = false;
        for (size_t i = 0; i < all_bboxes.size(); ++i)
        {
            // Ignore removed bboxes
            if (erased_bbox[i])
            {
                continue;
            }

            for (size_t j = i + 1; j < all_bboxes.size(); ++j)
            {
                // Skip removed bboxes
                if (erased_bbox[j])
                {
                    continue;
                }
                num_of_operations++;

                if ((all_bboxes[i] & all_bboxes[j]).area() > 0)
                {
                    // If the intersection of bboxes[i] and bboxes[j] is not empty
                    all_bboxes[i] |= all_bboxes[j]; // Set bboxes[i] to the union of bboxes[i] and bboxes[j]
                    erased_bbox[j] = true;          // Remove this bbox from future consideration
                    combined = true;
                }
            }
            if (combined)
            {
                break;
            }
        }
    } while (combined);

    // Create a new vector to hold the elements we want to keep
    // Also conform the boxes to our screen now
    std::vector<myRect> all_boxes_clean;

    for (int i = all_bboxes.size() - 1; i >= 0; i--)
    {
        if (!erased_bbox[i])
        {
            all_boxes_clean.push_back(myRect(FixBBox(all_bboxes[i], foreground.size())));
        }
    }

    all_bboxes = all_boxes_clean;

    // Drop all bounding boxes that are smaller than designated size
    // Also remove extremely large bounding boxes
    for (auto it = all_bboxes.begin(); it != all_bboxes.end();)
    {
        if (it->width < fg_bbox_minsize || it->height < fg_bbox_minsize ||
            it->width > fg_max_bbox_dimension || it->height >= fg_max_bbox_dimension)
        {
            it = all_bboxes.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // Clear all pixels outside our bounding boxes
    // Is this really needed? Looks like the sparkles and noise risks being added to the tracked image, thus we do want to remove all noise
    cv::Mat bb_mask = cv::Mat::zeros(fg_mask.size(), CV_8U);
    for (const auto &bbox : all_bboxes)
    {
        bb_mask(bbox).setTo(cv::Scalar(255));
    }
    fg_mask.setTo(cv::Scalar(0), bb_mask == 0);

    // Create our foreground image based on the foreground mask
    foreground.setTo(cv::Scalar(0));
    croppedFrame.copyTo(foreground, fg_mask);
    // foreground.setTo(cv::Scalar(0), bb_mask == 0 );

    //    cv::imshow( "new_fg_mask", fg_mask);
}

// Heal the background by slowly averaging areas where there is no robot lock
// We will average in new frames to the background to get the average background image
// Ideally we don't want to average in activelly tracked robots, however we need a mechanism
// to remove invalid areas that are tracked. We will thus implement a very slow decay for tracked
// areas, and a much faster decay for untracked areas, relying on the fact that a robot should move
// relativelly quickly.
void HeuristicOdometry::healBackground(cv::Mat &currFrame )
{
    // quit if we are not enabled
    if (!enable_background_healing)
    {
        return;
    }

    // If we haven't found both robots or either one is not tracking, don't heal
    if (!force_background_averaging && ((ourRobotTracker == NULL) || (opponentRobotTracker == NULL) ||
                                        (ourRobotTracker->numFramesNotTracked > 1) || (opponentRobotTracker->numFramesNotTracked > 1)))
    {
        return;
    }

    // NEed 16-bit version of our frame for averaging manipulations later
    cv::Mat currFrame_16bit;
    currFrame.convertTo(currFrame_16bit, CV_16U, 256);

    int myAveragingCount = averagingCount;
    int myTrackedAvgCount = trackedAvgCount; 

    // Fast untracked averaging
    cv::Mat newBackground_16bit = currBackground_16bit * ((myAveragingCount - 1.0) / myAveragingCount) + currFrame_16bit * 1.0 / myAveragingCount;

    // Slow tracked averaging
    cv::Mat newBackground_16bit_slow = currBackground_16bit * ((myTrackedAvgCount - 1.0) / myTrackedAvgCount) + currFrame_16bit * 1.0 / myTrackedAvgCount;

    // Robot locks get the slowest averaging
    // cv::Mat newBackground_16bit_ultraslow = currBackground_16bit*((robotAvgCount-1.0)/robotAvgCount) + currFrame_16bit*1.0/robotAvgCount;

    // Copy over the slow tracked areas
    // First do all bounding boxes that have not been removed
    for (const auto &bbox : all_bboxes)
    {
        cv::Rect fixedBBox = FixBBox(bbox, newBackground_16bit_slow);

        if (fixedBBox.area() > 0)
        {
            newBackground_16bit_slow(fixedBBox).copyTo(newBackground_16bit(fixedBBox));
        }
    }
    
    // Copy over the Robot areas
    for (auto trackedItem : _allRobotTrackers)
    {

        cv::Rect fixedBBox = FixBBox(trackedItem->bbox, newBackground_16bit_slow);

        if (fixedBBox.area() > 0)
        {
            currBackground_16bit(fixedBBox).copyTo(newBackground_16bit(fixedBBox));
        }
    }

    // Assign this image as the new background
    currBackground_16bit = newBackground_16bit;

    // Downscale image
    currBackground_16bit.convertTo(currBackground, CV_8U, 1.0 / 256);
}


void HeuristicOdometry::recreateBackgroundOnMatchStart(cv::Mat& currFrame)
{
    // First get the foreground mask from all tracked items
    cv::Mat tracked_fg_mask = cv::Mat::zeros(currFrame.size(), currFrame.type());

    for (auto currbbox : all_bboxes)
    {                
        fg_mask(currbbox).copyTo(tracked_fg_mask(currbbox));
    }

    // Next copy over entire currFrame to new background image
    cv::Mat newBackground = currFrame.clone();

    // Next overwrite tracked_fg_mask items from currBackground to newBackground
    currBackground.copyTo(newBackground, tracked_fg_mask);

    // Finaly make 16bit version
    currBackground = newBackground;
    currBackground.convertTo(currBackground_16bit, CV_16U, 256);
}

void HeuristicOdometry::ReinitBackground()
{
    regularBackground.copyTo(currBackground);
    currBackground.convertTo(currBackground_16bit, CV_16U, 256);
}

void HeuristicOdometry::MatchStartBackgroundInit(cv::Mat &currFrame)
{
    // First correct brightness of currframe
    correctBrightness(currFrame, true);

    // Make currframe the background
    currFrame.copyTo(currBackground);

    // Copy over the defined squares from our regular background
    cv::Rect leftrect(STARTING_LEFT_TL_x, STARTING_LEFT_TL_y, (STARTING_LEFT_BR_x - STARTING_LEFT_TL_x), (STARTING_LEFT_BR_y - STARTING_LEFT_TL_y));
    cv::Rect rightrect(STARTING_RIGHT_TL_x, STARTING_RIGHT_TL_y, (STARTING_RIGHT_BR_x - STARTING_RIGHT_TL_x), (STARTING_RIGHT_BR_y - STARTING_RIGHT_TL_y));

    float leftAdjust = FindOptimalBrightness(leftrect, currFrame);
    float rightAdjust = FindOptimalBrightness(rightrect, currFrame);

    // Adjust the brightness
    cv::Mat leftsquare = regularBackground(leftrect).clone() * leftAdjust * HEU_BRIGHTNESS_CORR;
    cv::Mat rightsquare = regularBackground(rightrect).clone() * rightAdjust* HEU_BRIGHTNESS_CORR;

    // Copy over the squares    
    leftsquare.copyTo(currBackground(leftrect));
    rightsquare.copyTo(currBackground(rightrect));

    imshow("Match Start Background", currBackground);

    currBackground.convertTo(currBackground_16bit, CV_16U, 256);
}

float HeuristicOdometry::GetForegroundSize(cv::Mat& inBackground, cv::Mat& inForeground)
{
       // First get mask for foreground extraction
    cv::Mat currBackgroundBlurred;
    cv::Mat croppedFrameBlurred;

    currBackgroundBlurred = inBackground.clone();
    croppedFrameBlurred = inForeground.clone();

    // blur
    for( int i = 0; i < blurCount; i++)
    {
        cv::blur(currBackgroundBlurred, currBackgroundBlurred, preMaskBlurSize);
        cv::blur(croppedFrameBlurred, croppedFrameBlurred, preMaskBlurSize);
    }

    // Get the absolute difference (leaving >0 values where there is a difference)
    cv::Mat foregroundBlurred;
    cv::absdiff(croppedFrameBlurred, currBackgroundBlurred, foregroundBlurred);

    cv::Mat myFgMask;
   // First get the mask that includes regions at least the minimum pixels away
    cv::threshold(foregroundBlurred, myFgMask, fg_threshold, 255, cv::THRESH_BINARY);

    cv::Mat diffThreshold = fg_threshold_ratio * currBackgroundBlurred;
    cv::Mat fg_maskRatio = foregroundBlurred > diffThreshold;
    cv::bitwise_and(fg_maskRatio, myFgMask, myFgMask);

    // return the average value
    return cv::mean(myFgMask)[0];
}

float HeuristicOdometry::FindOptimalBrightness(cv::Rect& myRect, cv::Mat& currFrame)
{
    float error = GetForegroundSize(regularBackground(myRect), currFrame(myRect));
    float brightnessAdjust = 1.0f;
    float brightnessscaler = 1.1f;
    bool increasing = true;

    float optimalBrigthness = 1.0f;

    // Find the global minima for brightness adjustment
    for( float brightnessAdjust = 0.3f; brightnessAdjust < 3.0f; brightnessAdjust *= 1.1f)
    {
        cv::Mat adjustedBG = regularBackground(myRect).clone();
        adjustedBG *= brightnessAdjust;
        float newerror = GetForegroundSize(adjustedBG, currFrame(myRect));

        if( newerror < error)
        {
            error = newerror;
            optimalBrigthness = brightnessAdjust;
        }
    }

    brightnessAdjust = optimalBrigthness;
    brightnessscaler = 1.03f;
    // Now fine tune global minima
    for(int i =0; i < 30; i++)
    {
        if( increasing)
        {
            brightnessAdjust = brightnessAdjust * brightnessscaler;
        }
        else
        {
            brightnessAdjust = brightnessAdjust / brightnessscaler;
        }

        // Get the Error
        cv::Mat adjustedBG = regularBackground(myRect).clone();
        adjustedBG *= brightnessAdjust;
        float newerror = GetForegroundSize(adjustedBG, currFrame(myRect));

        // Cap brightness adjust
        if( brightnessAdjust > 3.0f)
        {
            brightnessAdjust = 2.0f;
            increasing = false;
            continue;
        }
        
        if( brightnessAdjust < 0.3f)
        {
            brightnessAdjust = 0.3f;
            increasing = true;
            continue;
        }

        if( newerror > error)
        {
            increasing = !increasing;
            brightnessscaler = 1.0f + ((brightnessscaler-1.0f) / 5.0f);
            continue;
        }

        error = newerror;   
    }  

    return brightnessAdjust;
}


double lastTimeForBrightness = 0;
float brightnessCorrection = 1.0f;


void HeuristicOdometry::correctBrightness(cv::Mat &image, bool reset_averaging)
{
    // If disabled, dont do anything
    if( (IMAGE_INTENSITY_TIME_CONSTANT < 0) ||
        // If reference background isn't loaded then quit or background sane size
        refBackground.empty() ||  currBackground.empty() || currBackground.cols < IMAGE_INTENSITY_SQR_size || image.cols < IMAGE_INTENSITY_SQR_size
     )
    {
        refIntensities[0] = 0;
        refIntensities[1] = 0;
        refIntensities[2] = 0;
        refIntensities[3] = 0;
        brightnessCorrection = 1.0f;
        return;
    }


    // If we haven't calculated our reference background intensities yet, then do that now
    if(refIntensities[0] <= 0 )
    {
        // refBackground assumed to be black and white (1 channel)
        refIntensities[0] = cv::mean(refBackground(cv::Rect(IMAGE_INTENSITY_SQR_1_x, IMAGE_INTENSITY_SQR_1_y, IMAGE_INTENSITY_SQR_size, IMAGE_INTENSITY_SQR_size)))[0];
        refIntensities[1] = cv::mean(refBackground(cv::Rect(IMAGE_INTENSITY_SQR_2_x, IMAGE_INTENSITY_SQR_2_y, IMAGE_INTENSITY_SQR_size, IMAGE_INTENSITY_SQR_size)))[0];
        refIntensities[2] = cv::mean(refBackground(cv::Rect(IMAGE_INTENSITY_SQR_3_x, IMAGE_INTENSITY_SQR_3_y, IMAGE_INTENSITY_SQR_size, IMAGE_INTENSITY_SQR_size)))[0];
        refIntensities[3] = cv::mean(refBackground(cv::Rect(IMAGE_INTENSITY_SQR_4_x, IMAGE_INTENSITY_SQR_4_y, IMAGE_INTENSITY_SQR_size, IMAGE_INTENSITY_SQR_size)))[0];
    }

    // Get the time constant scaler 
    double deltaTime = currTime - lastTimeForBrightness;

    // Dont do anything for invalid time
    if( deltaTime <=0 )
    {
        return;
    }

    lastTimeForBrightness = currTime;

    float scaler = 0.0;
    if( !reset_averaging && (deltaTime < IMAGE_INTENSITY_TIME_CONSTANT && deltaTime>0))
    {
        scaler = 1.0 - deltaTime/IMAGE_INTENSITY_TIME_CONSTANT;
    }

    // Get the current correction factors for current frame
    double corrections[4] = {1.0, 1.0, 1.0, 1.0};
    corrections[0] = getBrightnessCorrection(image, IMAGE_INTENSITY_SQR_1_x, IMAGE_INTENSITY_SQR_1_y, IMAGE_INTENSITY_SQR_size, refIntensities[0]);
    corrections[1] = getBrightnessCorrection(image, IMAGE_INTENSITY_SQR_2_x, IMAGE_INTENSITY_SQR_2_y, IMAGE_INTENSITY_SQR_size, refIntensities[1]);
    corrections[2] = getBrightnessCorrection(image, IMAGE_INTENSITY_SQR_3_x, IMAGE_INTENSITY_SQR_3_y, IMAGE_INTENSITY_SQR_size, refIntensities[2]);
    corrections[3] = getBrightnessCorrection(image, IMAGE_INTENSITY_SQR_4_x, IMAGE_INTENSITY_SQR_4_y, IMAGE_INTENSITY_SQR_size, refIntensities[3]);

    // Sort it
    std::sort(corrections, corrections+4);

    // Get the average correction factor
    double newcorrection = (corrections[1] + corrections[2]) / 2.0;

    // Limit newcorrection
    newcorrection = (newcorrection < 0.02) ? 0.02 : newcorrection;
    newcorrection = (newcorrection > 40.0) ? 40.0 : newcorrection;
    
    // Calculate the time averaged brightness correction
    brightnessCorrection = brightnessCorrection * scaler + (1.0 - scaler)*newcorrection;

    // Now adjust the new image
    image.convertTo(image,  CV_16U, brightnessCorrection, 0); // Multiply by factor, no offset

    // Ensure pixel values are within [0, 255]
    cv::threshold(image, image, 255, 255, cv::THRESH_TRUNC); // Cap at 255

    // Set it back to 8 bit
    image.convertTo(image, CV_8U);
}

double HeuristicOdometry::getBrightnessCorrection(cv::Mat& src, int x, int y, int size , double refMean)
{
    if( x<0) { x= 0;}
    if( y<0) { y= 0;}   
    if( x+size >= src.cols) { x = src.cols - size - 1;}
    if( y+size >= src.rows) { y = src.rows - size - 1;}

    cv::Scalar srcMean = cv::mean(src(cv::Rect(x, y, size, size)));

    double srcBrightness = srcMean[0]; // Since it's grayscale, we use the first channel

    // Deal with special cases
    if( srcBrightness == 0 || refMean == 0) 
    {
        return 1.0;
    }

    return refMean / srcBrightness;
}

void HeuristicOdometry::calcShakeRemoval(cv::Mat &image)
{
    // If not enabled
    if (!enable_camera_antishake)
    {
        return;
    }

    // If background isn't properly initialized
    if (currBackground.cols < (x_start_region + antiShakeSize + 2 * crop_x) ||
        currBackground.rows < (y_start_region + antiShakeSize + 2 * crop_y))
    {
        return;
    }

    // Crop out the corners from the image and find where they should be
    // Do the area indicated
    cv::Rect cropparea(x_start_region, y_start_region, antiShakeSize, antiShakeSize);
    cv::Mat cropBgToFind = currBackground(cropparea);

    // Now crop the image for the region to scan
    cv::Rect cropimg(x_start_region, y_start_region - crop_y, antiShakeSize + 2 * crop_x, antiShakeSize + 2 * crop_y);
    cv::Mat cropImgToScan = image(cropimg);

    // cv::imshow("antishake",cropImgToScan );

    // Create the result matrix
    int result_cols = cropImgToScan.cols - cropBgToFind.cols + 1;
    int result_rows = cropImgToScan.rows - cropBgToFind.rows + 1;
    cv::Mat result(result_rows, result_cols, CV_32FC1);

    // Do the matching and normalize
    cv::matchTemplate(cropImgToScan, cropBgToFind, result, cv::TM_CCOEFF_NORMED);

    // Localizing the best match with minMaxLoc
    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

    // Set the y_offset and x_offset
    // x_offset = maxLoc.x;

    // If the shake is too much, then don't do it
    if ((abs(maxLoc.y - crop_y) <= max_shake_correction) && (abs(maxLoc.x - crop_x) <= max_shake_correction))
    {
        y_offset = maxLoc.y;
        x_offset = maxLoc.x;
    }
}
