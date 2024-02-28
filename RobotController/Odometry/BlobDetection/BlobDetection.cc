#include "BlobDetection.h"
#include "../../UIWidgets/ImageWidget.h"
#include "../../RobotConfig.h"

// Ctor
BlobDetection::BlobDetection(ICameraReceiver *videoSource) : OdometryBase(videoSource)
{
    // Initialize starting values
    SetAngle(0, false);
    SetAngle(0, true);
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
    VisionClassification robotData = DoBlobDetection(currFrame, _previousImage);

    // Now update our standard data
    UpdateData(robotData, frameTime);

    // Move currFrame to previousImage
    // But only if both our robot and opponent robot have been found, or we exceeded a timer

    if (robotData.GetRobotBlob() != nullptr || robotData.GetOpponentBlob() != nullptr || _prevImageTimer.getElapsedTime() > (1.0 / BLOBS_MIN_FPS))
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
VisionClassification BlobDetection::DoBlobDetection(cv::Mat &currFrame, cv::Mat &previousFrame)
{
    static ImageWidget motionImageWidget{"Motion", true};

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

    for (const cv::Rect &rect : potentialRobots)
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
    cv::Mat blobsImage;

    cv::cvtColor(thresholdImg, blobsImage, cv::COLOR_GRAY2BGR);
    for (const MotionBlob &blob : motionBlobs)
    {
        cv::rectangle(blobsImage, blob.rect, cv::Scalar(0, 255, 0), 2);
        cv::circle(blobsImage, blob.center, 5, cv::Scalar(0, 255, 0), 2);
    }

    // draw the potential robots
    motionImageWidget.UpdateMat(blobsImage);

    // Identify which blobs are our robots (finds robot positions)
    // classify the blobs and save them for later
    // Need previous data to be able to predict this data
    // Should we use extrapolated data? In this case maybe not so that we dont compound a bad reading?
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData lastDataUs = _prevDataRobot;
    OdometryData lastDataOpponent = _prevDataOpponent;
    locker.unlock();

    return _robotClassifier.ClassifyBlobs(motionBlobs, currFrame, thresholdImg, lastDataUs, lastDataOpponent);
}

void BlobDetection::UpdateData(VisionClassification robotBlobData, double timestamp)
{
    // Get unique access
    std::unique_lock<std::mutex> locker(_updateMutex);

    MotionBlob* robot = robotBlobData.GetRobotBlob();
    MotionBlob* opponent = robotBlobData.GetOpponentBlob();

    if (robot != nullptr && _IsValidBlob(*robot, _currDataRobot))
    {
        // Make a copy of currData for velocity calls
        _prevDataRobot = _currDataRobot;

        _currDataRobot.id++;             // Increment frame id
        _currDataRobot.time = timestamp; // Set to new time

        // Clear curr data
        _currDataRobot.Clear();
        _currDataRobot.isUs = true; // Make sure this is set
        _currDataRobot.userDataDouble["blobArea"] = robot->rect.area();

        // Update our robot position/velocity/angle
        SetData(robotBlobData.GetRobotBlob(), _currDataRobot, _prevDataRobot);
    }
    else
    {
        // increase invalid count + mark as invalid
        _currDataRobot.robotPosValid = _currDataRobot.userDataDouble["invalidCount"] < 10;
        _currDataRobot.userDataDouble["invalidCount"]++;
    }

    if (opponent != nullptr && _IsValidBlob(*opponent, _currDataOpponent))
    {
        // Make a copy of currData for velocity calls
        _prevDataOpponent = _currDataOpponent;

        _currDataOpponent.id++;             // Increment frame id
        _currDataOpponent.time = timestamp; // Set to new time

        // Clear curr data
        _currDataOpponent.Clear();
        _currDataOpponent.isUs = false; // Make sure this is set
        _currDataOpponent.userDataDouble["blobArea"] = opponent->rect.area();

        // Update opponent position/velocity info
        SetData(robotBlobData.GetOpponentBlob(), _currDataOpponent, _prevDataOpponent);
    }
    else
    {
        _currDataOpponent.robotPosValid = _currDataOpponent.userDataDouble["invalidCount"] < 10;
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
bool BlobDetection::_IsValidBlob(MotionBlob &blobNew, OdometryData &prevData)
{
    double blobArea = blobNew.rect.area();
    double numUpdatesInvalid = prevData.userDataDouble["invalidCount"];
    double lastBlobArea = prevData.userDataDouble["blobArea"];
    // it's possible the motion detector's blob shrunk due to detecting only a portion 
    bool invalidBlob = blobArea < lastBlobArea * 0.8 && numUpdatesInvalid < 10;

    return !invalidBlob;
}
// bool BlobDetection::_IsValidBlob(MotionBlob &blobNew, OdometryData &currData, OdometryData &prevData)
// {
//     double blobArea = blobNew.rect.area();

//     // Get previous data. If they dont exist, it will return 0.
//     double numUpdatesInvalid = prevData.userDataDouble["invalidCount"];
//     double lastBlobArea = prevData.userDataDouble["blobAarea"];

//     bool invalidBlob = blobArea < lastBlobArea * 0.8 && numUpdatesInvalid < 10;

//     // if the blob is too small and we haven't had too many invalid blobs
//     if (invalidBlob)
//     {
//         // we are invalid, so increment the number of invalid blobs
//         numUpdatesInvalid++;
//     }
//     else
//     {
//         // reset the number of invalid blobs
//         numUpdatesInvalid = 0;
//         lastBlobArea = blobArea;
//     }

//     currData.userDataDouble["invalidCount"] = numUpdatesInvalid;
//     currData.userDataDouble["blobArea"] = lastBlobArea;

//     return !invalidBlob;
// }

/**
 * @brief our position and velocity using just visual data
 * @param blob - the MotionBlob to update to
 * @param currData - the current data to update (will set the position and velocity)
 */
void BlobDetection::SetData(MotionBlob *blob, OdometryData &currData, OdometryData &prevData)
{
    double elapsedSincePrev = Clock::programClock.getElapsedTime() - prevData.time;

    //////////////////////// POS ////////////////////////
    // use the blob's center for the visual position
    currData.robotPosValid = true;
    currData.robotPosition = (*blob).center;
    currData.rect = (*blob).rect;

    //////////////////////// VEL ////////////////////////
    _GetSmoothedVisualVelocity(currData, prevData);

    //////////////////////// ANGLE ////////////////////////
    CalcAnglePathTangent(currData, prevData);
}

/**
 * Smooths out the visual velocity so it's not so noisy
 */
#define NEW_VISUAL_VELOCITY_TIME_WEIGHT_MS 250
#define ANGLE_SMOOTHING_TIME_CONSTANT 50
#define NEW_VISUAL_VELOCITY_WEIGHT_DIMINISH_OPPONENT 1
void BlobDetection::_GetSmoothedVisualVelocity(OdometryData &currData, OdometryData &prevData)
{
    // If the previous or current position isnt valid or the total elapsed time is too long, then restart velocity
    double elapsedVelTime = currData.time - prevData.userDataDouble["lastVelTime"];

    // If elapsed time between vel updates is too big, reset it
    if (elapsedVelTime > 0.1f)
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
void BlobDetection::CalcAnglePathTangent(OdometryData &currData, OdometryData &prevData)
{
    Angle retAngleRad;
    cv::Point2f lastAngleUpdatePos(prevData.userDataDouble["lastAnglePosX"], prevData.userDataDouble["lastAnglePosY"]);
    double lastAngleTime = prevData.userDataDouble["lastAngleTime"];
    double elapsedAngleTime = currData.time - lastAngleTime;

    // Save it back into the new point
    currData.userDataDouble["lastAnglePosX"] = lastAngleUpdatePos.x;
    currData.userDataDouble["lastAnglePosY"] = lastAngleUpdatePos.y;
    currData.userDataDouble["lastAngleTime"] = lastAngleTime;

    currData.robotAngleValid = false;

    // Save the old angle into the current one
    if (!prevData.robotAngleValid || std::isnan(prevData.robotAngle))
    {
        currData.robotAngle = Angle(0);
        currData.robotAngleVelocity = 0;
    }
    else
    {
        currData.robotAngleValid = true;
        currData.robotAngle = prevData.robotAngle;
        currData.robotAngleVelocity = prevData.robotAngleVelocity;
    }

    // If currPosition isn't valid then angle isn't valid either
    if (!currData.robotPosValid)
    {
        return;
    }

    // If the robot hasn't moved far enough to calculate new angle, then preserve old value
    cv::Point2f delta = currData.robotPosition - lastAngleUpdatePos;
    if (norm(delta) < DIST_BETWEEN_ANG_UPDATES_PX)
    {
        return;
    }

    // update the angle
    retAngleRad = Angle(atan2(delta.y, delta.x));
    if (std::isnan((double)retAngleRad))
    {
        retAngleRad = Angle(0);
    }

    // if the angle is closer to 180 degrees to the last angle
    if (abs(Angle(retAngleRad + M_PI - currData.robotAngle)) < abs(Angle(retAngleRad - currData.robotAngle)))
    {
        // add 180 degrees to the angle
        retAngleRad = Angle(retAngleRad + M_PI);
    }

    // Calculate angular velocity
    if (prevData.robotAngleValid && !std::isnan(prevData.robotAngle))
    {
        double weight = elapsedAngleTime * 1000 / ANGLE_SMOOTHING_TIME_CONSTANT;
        weight = min(weight, 1.0f);

        // Record the angle, but smooth it
        currData.robotAngle = prevData.robotAngle * (1.0 - weight) + retAngleRad * weight;

        double angularVelocity = (retAngleRad - prevData.robotAngle) / (currData.time - lastAngleTime);

        // Average it
        weight = ((currData.isUs) ? 1.0f : 0.8f) * elapsedAngleTime * 1000 / NEW_VISUAL_VELOCITY_TIME_WEIGHT_MS;

        weight = min(weight, 1.0f);

        // interpolate towards the visual angle velocity so it's not so noisy
        if (std::isnan(prevData.robotAngleVelocity))
        {
            currData.robotAngleVelocity = angularVelocity;
        }
        else
        {
            currData.robotAngleVelocity = prevData.robotAngleVelocity * (1.0 - weight) + weight * (angularVelocity - prevData.robotAngleVelocity);
        }
    }
    else
    {
        currData.robotAngle = retAngleRad;
        currData.robotAngleVelocity = 0;
    }

    // save the last position
    currData.userDataDouble["lastAnglePosX"] = currData.robotPosition.x;
    currData.userDataDouble["lastAnglePosY"] = currData.robotPosition.y;
    currData.userDataDouble["lastAngleTime"] = currData.time;
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
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;
    odoData.robotPosition = newPos;
    odoData.robotPosValid = true;

    OdometryData &odoData2 = (opponentRobot) ? _prevDataOpponent : _prevDataRobot;
    odoData2.robotPosition = newPos;
    odoData2.robotPosValid = true;
}

void BlobDetection::SetVelocity(cv::Point2f newVel, bool opponentRobot)
{
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;
    odoData.robotVelocity = newVel;

    OdometryData &odoData2 = (opponentRobot) ? _prevDataOpponent : _prevDataRobot;
    odoData2.robotVelocity = newVel;
}

void BlobDetection::SetAngle(double newAngle, bool opponentRobot)
{
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData &odoData = (opponentRobot) ? _currDataOpponent : _currDataRobot;

    odoData.robotAngle = Angle(newAngle);
    odoData.robotAngleValid = true;
    odoData.robotAngleVelocity = 0;

    OdometryData &odoData2 = (opponentRobot) ? _prevDataOpponent : _prevDataRobot;

    odoData2.robotAngle = Angle(newAngle);
    odoData2.robotAngleValid = true;
    odoData2.robotAngleVelocity = 0;
}
