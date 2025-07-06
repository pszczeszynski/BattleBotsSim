#include "BlobDetection.h"
#include "../../RobotConfig.h"
#include "../../RobotController.h"
#include "../../SafeDrawing.h"
#include "../../UIWidgets/ImageWidget.h"

// *********************************************************************
// NOTE:
// Blob detection is used for position and velocity of us and them.
// It does do path tangent calculation here, but it is not used for anything yet.
// **********************************************************************

// Ctor
BlobDetection::BlobDetection(ICameraReceiver* videoSource) : OdometryBase(videoSource)
{
    // Initialize starting values
    SetAngle(Angle(0), false, 0, 0, false);
    SetAngle(Angle(0), true, 0, 0, false);
    SetPosition(cv::Point2f(0, 0), false);
    SetPosition(cv::Point2f(0, 0), true);
    SetVelocity(cv::Point2f(0, 0), false);
    SetVelocity(cv::Point2f(0, 0), true);
}

// Run the main blob detection algorithm
void BlobDetection::_ProcessNewFrame(cv::Mat currFrame, double frameTime)
{

    // If previous image is empty, then initialize it to the current frame
    if (_previousImage.empty())
    {
        _previousImage = currFrame;
        prevFrameTime = frameTime;
        return;
    }

    // Do Blob detection and figure out which blobs is us versus opponent
    // Defer blocking of locker until inside DoBlobDetection core so that we give more chances to update data by other
    // threads
    std::unique_lock<std::mutex> locker(_updateMutex, std::defer_lock);
    VisionClassification robotData = DoBlobDetection(currFrame, _previousImage, locker, frameTime); // Locks the locker

    // Now update our standard data
    UpdateData(robotData, frameTime);
    _tempData = _currDataRobot;

    locker.unlock();

    // Move currFrame to previousImage
    // But only if both our robot and opponent robot have been found, or we exceeded a timer

    if (robotData.GetRobotBlob() != nullptr || robotData.GetOpponentBlob() != nullptr ||
        _prevImageTimer.getElapsedTime() > (1.0 / BLOBS_MIN_FPS))
    {
        // save the current frame as the previous frame
        _previousImage = currFrame;
        _prevImageTimer.markStart();
    }
}

/**
 * Locates the robots in the frame
 * Note: Incoming images are black and white
 */
VisionClassification BlobDetection::DoBlobDetection(cv::Mat& currFrame, cv::Mat& previousFrame,
                                                    std::unique_lock<std::mutex>& locker, double frameTime)
{

    // hyperparameters
    const cv::Size BLUR_SIZE = cv::Size(14, 14);
    const float MIN_AREA = pow(min(MIN_OPPONENT_BLOB_SIZE, MIN_ROBOT_BLOB_SIZE), 2);
    const float MAX_AREA = pow(max(MAX_OPPONENT_BLOB_SIZE, MAX_ROBOT_BLOB_SIZE), 2);
    const int BLOB_SEARCH_SIZE = 10;

    // Compute the absolute difference between the current frame and the previous frame
    cv::Mat diff;
    cv::absdiff(previousFrame, currFrame, diff);

    // Convert the difference to grayscale
    cv::Mat grayDiff;

    if (diff.channels() == 1)
    {
        grayDiff = diff;
    }
    else if (diff.channels() == 3)
    {
        cv::cvtColor(diff, grayDiff, cv::COLOR_BGR2GRAY);
    }
    else if (diff.channels() == 4)
    {
        cv::cvtColor(diff, grayDiff, cv::COLOR_BGRA2GRAY);
    }

    // check if neural valid
    CVPosition& neuralOdometry = RobotController::GetInstance().odometry.GetNeuralOdometry();
    OdometryData neuralData = neuralOdometry.GetData();
    bool blackOutNeural = false;

    if (neuralOdometry.IsRunning() && neuralData.robotPosValid && neuralData.GetAge() < 0.1)
    {
        std::vector<int> boundingBox = neuralOdometry.GetBoundingBox();
        boundingBox[0] -= boundingBox[2] / 2;
        boundingBox[1] -= boundingBox[3] / 2;
        blackOutNeural = true;
        // imshow
        cv::Rect neuralRect = cv::Rect(boundingBox[0], boundingBox[1], boundingBox[2], boundingBox[3]);
        // extrapolate
        neuralRect.x += neuralData.robotVelocity.x * (Clock::programClock.getElapsedTime() - neuralData.time);
        neuralRect.y += neuralData.robotVelocity.y * (Clock::programClock.getElapsedTime() - neuralData.time);

        cv::rectangle(grayDiff, neuralRect, cv::Scalar(0), -1);
    }

    // cv::cvtColor(diff, grayDiff, cv::COLOR_BGR2GRAY); // Image is already grayscale

    // Convert the difference to a binary image with a certain threshold
    cv::Mat thresholdImg;
    cv::threshold(grayDiff, thresholdImg, MOTION_LOW_THRESHOLD, 255, cv::THRESH_BINARY);

    // blurr and re-thresh to make it more leanient
    cv::blur(thresholdImg, thresholdImg, BLUR_SIZE);
    cv::threshold(thresholdImg, thresholdImg, 15, 255, cv::THRESH_BINARY);

    // find big blobs in the image using a blob detector

    // iterate through every pixel in the image and find the largest blob
    std::vector<cv::Rect> potentialRobots = {};
    for (int y = 0; y < thresholdImg.rows; y += BLOB_SEARCH_SIZE)
    {
        for (int x = 0; x < thresholdImg.cols; x += BLOB_SEARCH_SIZE)
        {
            // if this pixel is white, then it is part of a blob
            if (thresholdImg.at<uchar>(y, x) == 255)
            {
                // flood fill, mark the blob as 100 so that we don't flood fill it again
                cv::Rect rect;
                cv::floodFill(thresholdImg, cv::Point(x, y), cv::Scalar(100), &rect);

                // if the blob is a reasonable size, add it to the list
                if (rect.area() >= MIN_AREA && rect.area() <= MAX_AREA)
                {
                    // add the rect to the list of potential robots
                    potentialRobots.push_back(rect);
                }
            }
        }
    }

    // for each robot, find the EXACT center of the robot by counting the white pixels in the blob and averaging them
    std::vector<MotionBlob> motionBlobs = {};

    for (const cv::Rect& rect : potentialRobots)
    {
        // find the average of all the white pixels in the blob
        int numWhitePixels = 0;
        cv::Point2f averageWhitePixel = cv::Point2f(0, 0);
        for (int y = rect.y; y < rect.y + rect.height; y++)
        {
            for (int x = rect.x; x < rect.x + rect.width; x++)
            {
                // if this pixel is white, then add it to the average
                if (thresholdImg.at<uchar>(y, x) > 0)
                {
                    averageWhitePixel += cv::Point2f(x, y);
                    numWhitePixels++;
                }
            }
        }
        // divide by the number of white pixels to get the average
        averageWhitePixel /= numWhitePixels;

        // add the average to the list of robot centers
        motionBlobs.emplace_back(MotionBlob{rect, averageWhitePixel, &currFrame});
    }

    // draw the blobs
    std::unique_lock<std::mutex> debuglock(_mutexDebugImage); // Locks the mutex
    // _debugImage = cv::Mat::zeros(thresholdImg.size(), CV_8UC1); // Initialize debug image to black

    // cv::cvtColor(thresholdImg, _debugImage, cv::COLOR_BGR2GRAY);
    thresholdImg.copyTo(_debugImage); // Copy the threshold image to the debug image

    for (const MotionBlob& blob : motionBlobs)
    {
        cv::rectangle(_debugImage, blob.rect, cv::Scalar(255), 2);
        safe_circle(_debugImage, blob.center, 5, cv::Scalar(255), 2);
    }

    // draw the potential robots
    // motionImageWidget.UpdateMat(blobsImage);

    // static ImageWidget motionImageWidget{"Motion", true};
    debuglock.unlock(); // Unlock the mutex

    // Identify which blobs are our robots (finds robot positions)
    // classify the blobs and save them for later
    // Need previous data to be able to predict this data
    // Should we use extrapolated data? In this case maybe not so that we dont compound a bad reading?
    locker.lock();
    return _robotClassifier.ClassifyBlobs(motionBlobs, currFrame, thresholdImg, _currDataRobot, _currDataOpponent,
                                          blackOutNeural, frameTime);
}

void BlobDetection::GetDebugImage(cv::Mat& debugImage, cv::Point offset)
{
    OdometryBase::GetDebugImage(debugImage, offset); // Call base class to add the odometry data

    // Get unique access to _debugImage
    std::unique_lock<std::mutex> locker(_mutexDebugImage);
    if (_debugImage.empty() || _debugImage.size() != debugImage.size() || _debugImage.type() != debugImage.type())
    {
        locker.unlock();
        return; // No valid _debugImage to merge
    }

    // Add _debugImage to debugImage
    cv::add(debugImage, _debugImage, debugImage); // Add images, store result in debugImage

    locker.unlock(); // Unlock mutex after operation
}

#ifdef SIMULATION
// #define HARDCODE_SIM
#endif

void BlobDetection::UpdateData(VisionClassification robotBlobData, double timestamp)
{
    // Get unique access (already locked from calling function)
    //  std::unique_lock<std::mutex> locker(_updateMutex);

#ifndef HARDCODE_SIM
    MotionBlob* robot = robotBlobData.GetRobotBlob();
    MotionBlob* opponent = robotBlobData.GetOpponentBlob();
#else
    // for hardcoding sim:
    MotionBlob robot;
    MotionBlob opponent;
    robot.center = robotPosSim;
    opponent.center = opponentPosSim;
#endif

    // Increment frame id whether we have a valid blob or not to tell
    // odometry algorithm to consider to result (whether positive or negative)
    _currDataRobot.id++; // Increment frame id
    _currDataRobot.frameID = frameID;
    _currDataRobot.time = timestamp; // Set to new time

#ifndef HARDCODE_SIM
    if (robot != nullptr && _IsValidBlob(*robot, _currDataRobot))
#else
    if (true)
#endif
    {
        // Clear curr data
        _currDataRobot.Clear();
        _currDataRobot.isUs = true; // Make sure this is set
#ifndef HARDCODE_SIM
        _currDataRobot.userDataDouble["blobArea"] = robot->rect.area();
        // Update our robot position/velocity/angle
        _SetData(robotBlobData.GetRobotBlob(), _currDataRobot, _prevDataRobot, _prevAngleDataRobot, timestamp);
#else
        // Update our robot position/velocity/angle
        _SetData(&robot, _currDataRobot, _prevDataRobot, timestamp);
#endif
    }
    else
    {

        // increase invalid count + mark as invalid
        double invalidCount = _currDataRobot.userDataDouble["invalidCount"];

        if (invalidCount >= 10)
        {
            _currDataRobot.robotPosValid = false;
            _currDataRobot.InvalidateAngle();
            _prevDataRobot = _currDataRobot; // Invalidate old data
        }

        _currDataRobot.userDataDouble["invalidCount"]++;
    }

    // Increment id and frame
    _currDataOpponent.id++; // Increment frame id
    _currDataOpponent.frameID = frameID;
    _currDataOpponent.time = timestamp; // Set to new time

#ifndef HARDCODE_SIM
    if (opponent != nullptr && _IsValidBlob(*opponent, _currDataOpponent))
#else
    if (true)
#endif
    {
        // Make a copy of currData for velocity calls
        _prevDataOpponent = _currDataOpponent;

        // Clear curr data
        _currDataOpponent.Clear();
        _currDataOpponent.isUs = false; // Make sure this is set

#ifndef HARDCODE_SIM
        _currDataOpponent.userDataDouble["blobArea"] = opponent->rect.area();
        // Update opponent position/velocity info
        _SetData(robotBlobData.GetOpponentBlob(), _currDataOpponent, _prevDataOpponent, _prevAngleDataOpponent,
                 timestamp);
#else
        // Update opponent position/velocity info
        _SetData(&opponent, _currDataOpponent, _prevDataOpponent, _prevAngleDataOpponent, timestamp);
#endif
    }
    else
    {
        double invalidCount = _currDataOpponent.userDataDouble["invalidCount"];

        if (invalidCount >= 10)
        {
            _currDataOpponent.robotPosValid = false;
            _currDataOpponent.InvalidateAngle();
            _prevDataOpponent = _currDataOpponent; // Invalidate old data
        }

        _currDataOpponent.userDataDouble["invalidCount"]++;
    }
}

// the displacement required to update the angle
#define DIST_BETWEEN_ANG_UPDATES_PX 5

// 0 = no change, 1 = full change
#define MOVING_AVERAGE_RATE 1.0

// how fast the robot needs to be moving to update the angle
#define VELOCITY_THRESH_FOR_ANGLE_UPDATE 100 * WIDTH / 720.0

// max rotational speed to update the angle radians per second
#define MAX_ROTATION_SPEED_TO_ALIGN 250 * TO_RAD

/**
 * Makes sure the blob didn't shrink too much. Permitted to shrink if
 * we have too many invalid marks
 */
bool BlobDetection::_IsValidBlob(MotionBlob& blobNew, OdometryData& prevData)
{
    double blobArea = blobNew.rect.area();
    double numUpdatesInvalid = prevData.userDataDouble["invalidCount"];
    double lastBlobArea = prevData.userDataDouble["blobArea"];
    // it's possible the motion detector's blob shrunk due to detecting only a portion
    bool invalidBlob = blobArea < lastBlobArea * 0.8 && numUpdatesInvalid < 10;

    return !invalidBlob;
}

/**
 * @brief our position and velocity using just visual data
 * @param blob - the MotionBlob to update to
 * @param currData - the current data to update (will set the position and velocity)
 */
void BlobDetection::_SetData(MotionBlob* blob, OdometryData& currData, OdometryData& prevData,
                             OdometryData& prevAngleData, double timestamp)
{
    //////////////////////// POS ////////////////////////
    // use the blob's center for the visual position
    currData.robotPosValid = true;
    currData.robotPosition = (*blob).center;
    currData.rect = (*blob).rect;

    //////////////////////// VEL ////////////////////////
    _GetSmoothedVisualVelocity(currData, prevData);

    //////////////////////// ANGLE ////////////////////////
    CalcAnglePathTangent(currData, prevAngleData, timestamp);

    prevData = currData;
}

/**
 * Smooths out the visual velocity so it's not so noisy
 */
#define NEW_VISUAL_VELOCITY_TIME_WEIGHT_MS 50
#define NEW_VISUAL_VELOCITY_WEIGHT_DIMINISH_OPPONENT 1
void BlobDetection::_GetSmoothedVisualVelocity(OdometryData& currData, OdometryData& prevData)
{
    if (prevData.robotPosValid == false)
    {
        // Don't update velocity incase it was set by a setData call
        return;
    }

    // If the previous or current position isnt valid or the total elapsed time is too long, then restart velocity
    double elapsedVelTime = currData.time - prevData.userDataDouble["lastVelTime"];

    // If elapsed time between vel updates is too big, reset it
    if ((elapsedVelTime > 0.1f) || (currData.robotPosValid == false) || (prevData.robotPosValid == false))
    {
        currData.userDataDouble["lastVelTime"] = currData.time;
        currData.robotVelocity = cv::Point2f(0, 0);
        return;
    }

    // visual velocity
    cv::Point2f visualVelocity = (currData.robotPosition - prevData.robotPosition) / elapsedVelTime;

    // compute weight for interpolation. Reduce weight for opponents
    double weight = ((currData.isUs) ? 1.0f : 0.8f) * elapsedVelTime * 1000 / NEW_VISUAL_VELOCITY_TIME_WEIGHT_MS;

    weight = min(weight, 1.0f);

    // interpolate towards the visual velocity so it's not so noisy
    currData.robotVelocity = InterpolatePoints(prevData.robotVelocity, visualVelocity, weight);
    currData.userDataDouble["lastVelTime"] = currData.time;

    return;
}

/**
 * @brief updates the angle of the robot using the velocity
 */
void BlobDetection::CalcAnglePathTangent(OdometryData& currData, OdometryData& prevAngleData, double timestamp)
{
    constexpr float kAngleSmoothingTimeConstantMs = 80;

    double elapsedAngleTime = timestamp - prevAngleData.GetAngleFrameTime();
    cv::Point2f delta = currData.robotPosition - prevAngleData.robotPosition;

    // Check both distance and time thresholds
    if (norm(delta) < DIST_BETWEEN_ANG_UPDATES_PX || elapsedAngleTime < 0.001 || !currData.robotPosValid)
    {
        // Copy previous angle data
        currData.SetAngle(prevAngleData.GetAngle(), prevAngleData.GetAngleVelocity(), prevAngleData.GetAngleFrameTime(),
                          prevAngleData.IsAngleValid() && currData.robotPosValid);
        return;
    }


    // update the angle
    Angle newAngle = Angle(atan2(delta.y, delta.x));
    if (std::isnan((double)newAngle))
    {
        currData.SetAngle(Angle(0), 0, timestamp, false);
        return;
    }

    // if the angle is closer to 180 degrees to the last angle
    if (abs(Angle(newAngle + M_PI - prevAngleData.GetAngle())) < abs(Angle(newAngle - prevAngleData.GetAngle())))
    {
        // add 180 degrees to the angle
        newAngle = Angle(newAngle + M_PI);
    }

    Angle newAngleInterpolated{0};
    double angularVelocity{0};

    // Calculate angular velocity
    if (prevAngleData.IsAngleValid() && !std::isnan(prevAngleData.GetAngle()))
    {
        // Interpolate the angle
        double weight = min(elapsedAngleTime * 1000 / kAngleSmoothingTimeConstantMs, 1.0f);
        newAngleInterpolated = InterpolateAngles(prevAngleData.GetAngle(), newAngle, weight);
        angularVelocity = (newAngleInterpolated - prevAngleData.GetAngle()) / elapsedAngleTime;
    }

    currData.SetAngle(newAngleInterpolated, angularVelocity, timestamp, true);
    prevAngleData = currData;
}

void BlobDetection::SwitchRobots(void)
{
    // Switch who's who
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData temp_Robot = _currDataRobot;
    _currDataRobot = _currDataOpponent;
    _currDataOpponent = temp_Robot;

    _currDataRobot.isUs = true;
    _currDataOpponent.isUs = false;

    temp_Robot = _prevDataRobot;
    _prevDataRobot = _prevDataOpponent;
    _prevDataOpponent = temp_Robot;

    _prevDataRobot.isUs = true;
    _prevDataOpponent.isUs = false;
}

void BlobDetection::SetPosition(cv::Point2f newPos, bool opponentRobot)
{
    constexpr float IGNORE_THRESH_PX = 10;
    std::unique_lock<std::mutex> locker(_updateMutex);

    // ignore the request if our existing position is fine
    if (cv::norm(newPos - _currDataRobot.robotPosition) < IGNORE_THRESH_PX)
    {
        return;
    }

    OdometryData& odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;
    odoData.robotPosition = newPos;
    odoData.robotVelocity = cv::Point2f(0, 0);
    odoData.robotPosValid = true;
    odoData.time = Clock::programClock.getElapsedTime();
    odoData.id++;

    OdometryData& prevData = (opponentRobot) ? _prevDataOpponent : _prevDataRobot;
    prevData.robotPosition = newPos;
    prevData.robotVelocity = cv::Point2f(0, 0);
    prevData.robotPosValid = false;
}

void BlobDetection::SetVelocity(cv::Point2f newVel, bool opponentRobot)
{
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData& odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;
    odoData.robotVelocity = newVel;
    odoData.id++;

    OdometryData& odoData2 = (opponentRobot) ? _prevDataOpponent : _prevDataRobot;
    odoData2.robotVelocity = newVel;
}

void BlobDetection::SetAngle(Angle newAngle, bool opponentRobot, double angleFrameTime, double newAngleVelocity,
                             bool valid)
{
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData& odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;
    odoData.SetAngle(newAngle, newAngleVelocity, angleFrameTime, valid);
    odoData.id++;

    OdometryData& odoData2 = (opponentRobot) ? _prevDataOpponent : _prevDataRobot;
    odoData2.SetAngle(newAngle, newAngleVelocity, angleFrameTime, valid);

    OdometryData& odoData3 = (opponentRobot) ? _prevAngleDataOpponent : _prevAngleDataRobot;
    odoData3.SetAngle(newAngle, newAngleVelocity, angleFrameTime, valid);
}
