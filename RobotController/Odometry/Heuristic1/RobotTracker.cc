#include "RobotTracker.h"

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
#include "../../MathUtils.h"

// Initialize static variables
bool RobotTracker::useMultithreading = true;
int RobotTracker::numberOfThreads = 8.0;
int RobotTracker::maxNewBBoxIncrease = 40;                  // Number of pixels new box can grow by
float RobotTracker::robotVelocitySmoothing = 0.2;           // (s) smoothing of the velocity projection
float RobotTracker::detectionVelocitySmoothing = 1.0;       // (s) smoothing of the background detection
float RobotTracker::minVelocity = 7.0;                      // Min number of pixels/s movement required
float RobotTracker::moveTowardsCenter = 40.0;               // Amount of pixels per second to move towards center (?20?)
float RobotTracker::rotateTowardsMovement = 0;              // Amount of radians per second to move towards velocity dir (?2?)
float RobotTracker::rotateTowardsWeight = 1.0 / 50.0;       // Scaling factor to increase weight vs speed
float RobotTracker::minSpeedForRotationCorrection = 30.0;   // Minimum speed in pixels/s before we add in movement
float RobotTracker::bboxFoundIsMuchSmallerThreshold = 0.75; // The area reduction in bounding box that will trigger regeneration of Foreground
int RobotTracker::combinedBBoxScanBuffer = 15;              // Number of pixels to grow extrapolated bbox by to scan a combined bbox with
bool RobotTracker::matchingAreaAddOurBBoxes = true;         // increase matching area to always include our bbox and predicted bbox
bool RobotTracker::derateResultsByDistance = true;
float RobotTracker::distanceDerating_start = 1.0;     // The mutliplier to results at the expected location
float RobotTracker::distanceDerating_stop = 0.5;      // The maximum derating for things far away
float RobotTracker::distanceDerating_distance = 15.0; // The radial distance at which we reach _stop intensity
int RobotTracker::distanceDeratingMatSize = 999;      // Width/Height. Large enough to ensure we never run out. Most we will every use is mayb 100x100.

// Find Pos and Rotation using Match TemplateSearch. This will be done using parallel operation
float RobotTracker::deltaAngleSweep = 8;  // Delta +/- angle to sweep
float RobotTracker::deltaAngleStep = 0.5; // Number of degrees to step angle sweep
int RobotTracker::matchBufffer = 10;      // number of pixels around all edges to expand search (fixed pixels)

// **********************************
// General Purpose Functions
// **********************************
// Returns the area of the overlap


int FindBBoxWithLargestOverlap(const std::vector<myRect> &allBBoxes, const cv::Rect &targetBBox, cv::Rect &bestBBox, int &indexFound)
{
    // Go through all the bboxes and find the one that overlaps us the most
    indexFound = -1;
    int bestArea = 0;

    // Lets compare the area to our current position and our extrapolated position
    for (int index = 0; index < allBBoxes.size(); index++)
    {
        // Check extrapolated position
        int newarea = (targetBBox & allBBoxes[index]).area();

        if ((newarea > 0) && (newarea > bestArea))
        {
            indexFound = index;
            bestBBox = allBBoxes[index];
            bestArea = newarea;
        }
    }

    return bestArea;
}

// Returns the closest distance and stores the best bbox
float FindClosestBBox(const std::vector<myRect> &allBBoxes, const cv::Point2f &point, cv::Rect &bestBBox, int &indexFound)
{
    // Go through all the bboxes and find the one whos center is the closes
    indexFound = -1;
    float closest = -1;

    // Lets compare the distance
    for (int index = 0; index < allBBoxes.size(); index++)
    {
        // Check the distance
        float newDistance = distance(GetRectCenter(allBBoxes[index]), point);

        if ((closest < 0) || (newDistance < closest))
        {
            indexFound = index;
            closest = newDistance;
            bestBBox = allBBoxes[index];
        }
    }

    return closest;
}

// Generic function to print text on the image for debugging
void printText(std::string text, cv::Mat &image, int yoffset, int xoffset)
{
    cv::Point org(xoffset, yoffset); // bottom-left corner of the text string in the image
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.5;
    cv::Scalar color(255, 255, 255); // white color

    // Add the text to the image
    cv::putText(image, text, org, fontFace, fontScale, color);
}

std::mutex mutexMarkTime;
Clock timing_clock;
std::vector<double> timing_list;
std::vector<std::string> timing_text;

void markTime(std::string note, double time)
{
    mutexMarkTime.lock();

    if (time < 0)
    {
        time = timing_clock.getElapsedTime();
    }

    timing_list.push_back(time);
    timing_text.push_back(note);
    mutexMarkTime.unlock();
}


cv::Rect FixBBox(const cv::Rect &bbox, const cv::Mat &mat)
{
    cv::Rect matBBox(cv::Point(0, 0), mat.size());

    return bbox & matBBox;
}

cv::Rect FixBBox(const cv::Rect &bbox, const cv::Size &matSize)
{
    cv::Rect matBBox(cv::Point(0, 0), matSize);

    return bbox & matBBox;
}

cv::Point2f GetRectCenter(const cv::Rect &inbox)
{
    cv::Point2f center(inbox.x + inbox.width / 2.0, inbox.y + inbox.height / 2.0);
    return center;
}

cv::Point2f LimitPointToRect(cv::Rect &roi, cv::Point2f point)
{
    if (point.x < roi.x)
    {
        point.x = roi.x;
    }
    if (point.y < roi.y)
    {
        point.y = roi.y;
    }
    if (point.x > roi.x + roi.width)
    {
        point.x = roi.x + roi.width;
    }
    if (point.y > roi.y + roi.height)
    {
        point.y = roi.y + roi.height;
    }

    return point;
}

Vector2 LimitPointToRect(cv::Rect &roi, Vector2 point)
{
    if (point.x < roi.x)
    {
        point.x = roi.x;
    }
    if (point.y < roi.y)
    {
        point.y = roi.y;
    }
    if (point.x > roi.x + roi.width)
    {
        point.x = roi.x + roi.width;
    }
    if (point.y > roi.y + roi.height)
    {
        point.y = roi.y + roi.height;
    }

    return point;
}

cv::Rect getBoundingRect(const cv::Rect &rect1, const cv::Rect &rect2, int buffer)
{
    // Calculate the top left point of the bounding rectangle
    int x = min(rect1.x, rect2.x) - buffer;
    int y = min(rect1.y, rect2.y) - buffer;

    // Calculate the bottom right point of the bounding rectangle
    int x2 = max(rect1.x + rect1.width, rect2.x + rect2.width) + 2 * buffer;
    int y2 = max(rect1.y + rect1.height, rect2.y + rect2.height) + 2 * buffer;

    // Create the bounding rectangle
    cv::Rect boundingRect(x, y, x2 - x, y2 - y);

    return boundingRect;
}

// Moves the rect to be fully inside. Will reduce width/height only if required
cv::Rect MoveRectToBeInside(cv::Rect recttofix, cv::Mat &mat_to_fit)
{
    // Reduce width/height if required
    if (recttofix.width > mat_to_fit.cols)
    {
        recttofix.width = mat_to_fit.cols;
    }

    if (recttofix.height > mat_to_fit.rows)
    {
        recttofix.height = mat_to_fit.rows;
    }

    // Fix x
    if (recttofix.x < 0)
    {
        recttofix.x = 0;
    }
    else if (recttofix.x + recttofix.width > mat_to_fit.cols)
    {
        recttofix.x -= recttofix.x + recttofix.width - mat_to_fit.cols;
    }

    // Fix y
    if (recttofix.y < 0)
    {
        recttofix.y = 0;
    }
    else if (recttofix.y + recttofix.height > mat_to_fit.rows)
    {
        recttofix.y -= recttofix.y + recttofix.height - mat_to_fit.rows;
    }

    return recttofix;
}

// RotateSize: returns the increased size if rotated by degrees, ideally < 90 degrees
cv::Size RotateSize(cv::Size startSize, float degrees)
{
    float fixeddeg = abs(degrees);
    float newwidth = cos(deg2rad(fixeddeg)) * startSize.width + sin(deg2rad(fixeddeg)) * startSize.height;
    float newheight = cos(deg2rad(fixeddeg)) * startSize.height + sin(deg2rad(fixeddeg)) * startSize.width;

    return cv::Size(std::ceil(newwidth), std::ceil(newheight));
}

// Calculates the bbox of the provided mask. This is to help reduce bbox if theres a lot of black border
cv::Rect CalculateMaskBBox(const cv::Mat &mask)
{
    // Find non-zero pixels
    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);

    // Calculate bounding box
    cv::Rect boundingBox = cv::boundingRect(points);

    return boundingBox;
}

// ****************************************
// Vector2
// ****************************************

// Return magnitude of the vector
float Vector2::mag(void)
{
    return std::sqrt(x * x + y * y);
}

cv::Point Vector2::Point(float scale)
{
    return cv::Point((int)(scale * x + 0.5), (int)(scale * y + 0.5));
}

float Vector2::angle(void)
{
    if (x == 0 && y == 0)
    {
        return 0;
    }

    return angleWrap(rad2deg(atan2f(y, x)));
}

float Vector2::angleRad(void)
{
    return deg2rad(angle());
}

cv::Point2f Vector2::Point2f()
{
    return cv::Point2f(x, y);
}
// ****************************************
// RobotTracker
// ****************************************
static int uniqueID = 1;

RobotTracker::RobotTracker(cv::Rect bboxin)
{
    id = uniqueID++;
    bbox = bboxin;
    position.x = bbox.x + bbox.width / 2.0;
    position.y = bbox.y + bbox.height / 2.0;

    // Initialize the distanceDerating box
    InitializeDeratingMat();
}

cv::Point RobotTracker::GetCenter(void)
{
    return cv::Point((int)position.x, (int)position.y);
}

// Returns the expected location of the bounding box
cv::Rect RobotTracker::GetExtrapolatedBBOX(double curr_time)
{
    cv::Rect new_bbox;
    new_bbox = bbox;
    double delta_time = curr_time - lastTime;

    new_bbox.x += delta_time * avgVelocity.x;
    new_bbox.y += delta_time * avgVelocity.y;

    return new_bbox;
}

// Returns the expected location of the bounding box
Vector2 RobotTracker::GetExtrapolatedPosition(double curr_time)
{
    double delta_time = curr_time - lastTime;

    Vector2 newpos = position;

    newpos.x += delta_time * avgVelocity.x;
    newpos.y += delta_time * avgVelocity.y;

    return newpos;
}

void RobotTracker::InitializeDeratingMat(void)
{
    // Create an empty image
    distanceDerating = cv::Mat(distanceDeratingMatSize, distanceDeratingMatSize, CV_32FC1);

    // Calculate the center of the image
    cv::Point center(distanceDeratingMatSize / 2, distanceDeratingMatSize / 2);

    // Iterate over each pixel in the image
    for (int y = 0; y < distanceDeratingMatSize; y++)
    {
        for (int x = 0; x < distanceDeratingMatSize; x++)
        {
            // Calculate the distance from the center
            float distance = cv::norm(cv::Point(x, y) - center);

            float intensity = (distanceDerating_distance - distance) / distanceDerating_distance * (distanceDerating_start - distanceDerating_stop) + distanceDerating_stop;

            if (intensity < distanceDerating_stop)
            {
                intensity = distanceDerating_stop;
            }

            // Set the intensity of the pixel
            distanceDerating.at<float>(y, x) = intensity;
        }
    }
}

// Find the best bbox and assigns it
void RobotTracker::FindBestBBox(double currTime, std::vector<myRect> &allBBoxes)
{
    // Clear bestBBox
    bestBBox = NULL;

    // Find the best overlap bboundingbox
    predictedBBox = GetExtrapolatedBBOX(currTime);
    cv::Rect bestBBox1(0, 0, 0, 0);

    int index1 = -1, index2 = -1;
    bool foundBBox = false;
    int area1 = FindBBoxWithLargestOverlap(allBBoxes, predictedBBox, bestBBox1, index1);
    cv::Rect bestBBox2(0, 0, 0, 0);
    int area2 = FindBBoxWithLargestOverlap(allBBoxes, bbox, bestBBox2, index2);

    // Go through all the bboxes and find the one that overlaps us the most
    if ((index1 >= 0) && (area1 > area2))
    {
        bestBBox = &allBBoxes[index1];
        bestBBox->numOfOwners++;
    }
    else if ((index2 >= 0) && (area2 >= area1))
    {
        bestBBox = &allBBoxes[index2];
        bestBBox->numOfOwners++;
    }
}

void RobotTracker::ProcessNewFrame(double currTime, cv::Mat &foreground, cv::Mat &currFrame, cv::Mat &new_fg_mask, int &doneInt, std::condition_variable_any &doneCV, std::mutex &mutex, cv::Mat &debugMat)
{
    // Setup cleanup code for muli-threading (increment counter when finished and notify all
    MultiThreadCleanup cleanup(doneInt, mutex, doneCV);
    debugLine = "";

    if (debug_DumpInfo)
    {
        debugImage = debugMat;
    }

    currTimeSaved = currTime;

    if (currTime > 2.0 * 41.855)
    {
        currTimeSaved = currTime;
    }

    // ****************************
    // Check for non-movement
    // Done at the beggining of the frame just to make sure it gets executed (doesn't have to be frame accurate)

    // Check velocity
    if (velForMovementDetection.mag() < minVelocity)
    {
        timeNotMoving += currTime - lastTime;
    }
    else
    {
        timeNotMoving -= currTime - lastTime; // Subtract 50ms from timeNOtMoving if moving
        if (timeNotMoving < 0)
        {
            timeNotMoving = 0;
        }
    }

    // ****************************************
    // Bounding Box Conditions
    //
    // 0) BBox found and is ok:
    //      -> Do minima search and update bounding box and foreground to new image
    //
    // 1) Bounding box not found:
    //     -> Extrapolate position based on velocity and return
    //     -> Keep old foreground
    //
    //
    // New Algorithm:
    // 2) If there are multiple robots wanting to use new foreground, then dont use new foreground (keep old one)
    //    and just keep our old image
    //

    // 2) Bounding box found but much smaller
    //     -> Image may have been clipped and foreground not identified correctly
    //     -> Grow bounding box to be at least as large as the current bounding box.
    //     -> After minima found, re-generate foreground using previous foreground as a guide
    //      --> Slowly shrink bbox towards the smaller one

    //
    // 3) Bounding box much much larger or bbox owned by more then 1 owner
    //     -> Bounding boxes most likely combined
    //     -> Find minima location
    //     -> How to extract ourselves from combined foreground? Options are:
    //        a) Keep old foreground, just move it and rotate it and assume this will be ok
    //        b) Keep old mask only (rotated+moved), and extract new foreground pixels
    //        c) Cut the new foreground with the BBOX of the old one in the new position

    // Case (1): no bounding box found
    if ((bestBBox == NULL) || bestBBox->empty())
    {
        // Extrapolate position based on velocity
        // Only doing position extrapolation, may want to add rotation as well in future
        bbox = predictedBBox;
        position = GetExtrapolatedPosition(currTime);
        lastTime = currTime;
        numFramesNotTracked++;
        debugLine += "Case 1";
        return;
    }

    cv::Rect matchingBBox = *bestBBox;
    bool useNewBBox = true;

    // Case (2): much smaller bounding box
    bool doFixPartialForeground = false;
    if (false && (matchingBBox.area() < bbox.area() * bboxFoundIsMuchSmallerThreshold))
    {
        // The foreground bbox is going to be increased before matching template to allow it to be scanned
        // Here we want to mark we want to keep our old bbox, although we may want to grow the width or height if new
        // bbox is larger, we also want to regen the foreground image
        doFixPartialForeground = true;
        useNewBBox = false;
        debugLine += "Case 2";
    }

    // ***************************
    // Case 3: If a big combined box is used, etract our best guess location from within it (or if owned by multiple owners)
    if (bestBBox->numOfOwners > 1)
    {
        useNewBBox = false;
        numFramesNotTracked++;
        debugLine += "Case 3a";
    }

    if ((matchingBBox.width - bbox.width > maxNewBBoxIncrease) || (matchingBBox.height - bbox.height > maxNewBBoxIncrease))
    {
        useNewBBox = false;
        numFramesNotTracked++;
        debugLine += "Case 3b";

        // Calculate the maximum area to scan of the foreground (minimize possibility of false convergence)
        matchingBBox = getBoundingRect(predictedBBox, bbox, combinedBBoxScanBuffer) & matchingBBox;
    }
    else // Otherwise if we're here then we are tracking bboxes
    {
        numFramesNotTracked = 0;
        debugLine += "Case 0";
    }

    // Grow the matchingBBox to always include starting and predicted bboxes
    if (matchingAreaAddOurBBoxes)
    {
        matchingBBox = getBoundingRect(matchingBBox, bbox);
        matchingBBox = getBoundingRect(matchingBBox, predictedBBox);
    }

    // General Positioning Algorithm
    // Use the old foreground to scan across the new foreground to see how much it moved
    Vector2 newRotation = rotation;

    double confidence = FindNewPosAndRotUsingMatchTemplate(currFrame, foreground, matchingBBox, newRotation);

    // Theres a small chance a solution wasnt found due to some faults.
    // This shouldn't ever happen, but just in case lets check for that
    if (sweep_rotation < -170)
    {
        // Do same extraplotion stuff as if we didn't find the bbox
        // Extrapolate position based on velocity
        // Only doing position extrapolation, may want to add rotation as well in future
        bbox = predictedBBox;
        position = GetExtrapolatedPosition(currTime);
        lastTime = currTime;
        numFramesNotTracked++;
        debugLine += "Failed FindNewPos";
        return;
    }

    // Calculate delta movement
    // finalBBox is the BBox referenced to the matchingBBox of the rotated old foreground
    cv::Point center_of_new_pos = cv::Point(finalBBox.x + finalBBox.width / 2, finalBBox.y + finalBBox.height / 2) + cv::Point(matchingBBox.x, matchingBBox.y);
    cv::Point center_of_old_pos(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    delta = center_of_new_pos - center_of_old_pos;

    // Move our position based on this delta
    Vector2 newPos(position.x + delta.x, position.y + delta.y);

    // If this position is outside the boundingbox, then clamp it
    // This generally can work really poorly if the bounding box position/shape changes a lot. Best not to do this
    // TBD: Maybe increase the movement towards center of BBox the further away it is?
    // newPos = LimitPointToRect( bestBBox, newPos);

    // Add in slow drift towards center
    double deltaTime = currTime - lastTime;
    if (deltaTime <= 0.000001)
    {
        deltaTime = 0.001;
    }

    double avgScaler = deltaTime / moveTowardsCenter;
    if (avgScaler <= 0.0)
    {
        avgScaler = 0.001f;
    }
    if (avgScaler > 1.0f)
    {
        avgScaler = 1.0f;
    }

    // Slowly move the point to the center of the bounding box
    newPos.x = newPos.x * (1.0f - avgScaler) + avgScaler * center_of_new_pos.x;
    newPos.y = newPos.y * (1.0f - avgScaler) + avgScaler * center_of_new_pos.y;
    // newPos.x += (center_of_new_pos.x>newPos.x) ?  moveTowardsCenter * deltaTime : -1.0 * moveTowardsCenter * deltaTime;
    // newPos.y += (center_of_new_pos.y>newPos.y) ?  moveTowardsCenter * deltaTime : -1.0 * moveTowardsCenter * deltaTime;

    // Calculate velocity
    Vector2 newVelocity((newPos.x - position.x) / deltaTime, (newPos.y - position.y) / deltaTime);

    avgVelocity.x = avgVelocity.x * (robotVelocitySmoothing - deltaTime) / robotVelocitySmoothing + deltaTime * newVelocity.x / robotVelocitySmoothing;
    avgVelocity.y = avgVelocity.y * (robotVelocitySmoothing - deltaTime) / robotVelocitySmoothing + deltaTime * newVelocity.y / robotVelocitySmoothing;

    // Calculate Rotation
    rotation = newRotation;

    // Move rotation towards velocity
    if (newVelocity.mag() > minSpeedForRotationCorrection)
    {
        // Get the angle difference between velocity and rotation
        float velAngle = atan2(newVelocity.y, newVelocity.x);
        float rotAngle = atan2(rotation.y, rotation.x);

        float deltaAngle = angleWrap(velAngle - rotAngle);

        // Add a scaled version of the error
        float scalingfactor = deltaTime * rotateTowardsMovement * rotateTowardsWeight * newVelocity.mag();

        rotAngle += scalingfactor * ((deltaAngle > 0) ? 1.0 : -1.0);

        rotation.x = cos(rotAngle);
        rotation.y = sin(rotAngle);
    }

    // Record new values
    lastTime = currTime;
    position = newPos;
    currVelocity = newVelocity;
    velForMovementDetection.x = velForMovementDetection.x * (detectionVelocitySmoothing - deltaTime) / detectionVelocitySmoothing + deltaTime * newVelocity.x / detectionVelocitySmoothing;
    velForMovementDetection.y = velForMovementDetection.y * (detectionVelocitySmoothing - deltaTime) / detectionVelocitySmoothing + deltaTime * newVelocity.y / detectionVelocitySmoothing;

    // CASE 2 situation (bounding box was much smaller)
    // Assign best BBox to new bbox, unless we are in special case where we want to keep our old one
    if (doFixPartialForeground)
    {
        FixPartialForeground(currFrame, foreground, new_fg_mask, *bestBBox);
    }
    else if (useNewBBox) // Case 0: Normal case
    {
        // Use the new bbox and foreground
        bbox = *bestBBox;
        fg_image = foreground(bbox).clone();
        fg_mask = new_fg_mask(bbox).clone();
    }
    else
    {
        // In this case we can't use the bbox, thus we need to instead use the matched images and not the new ones
        // First calculate the reduced bbox (rotation keeps growing it, so make sure its only as big as required)
        cv::Rect cropRect = CalculateMaskBBox(finalMask);

        // Crop the foreground and mask images to the new rect
        fg_image = finalImage(cropRect).clone();
        fg_mask = finalMask(cropRect).clone();

        // Move the cropRect to the final location
        cropRect.x += finalBBox.x + matchingBBox.x;
        cropRect.y += finalBBox.y + matchingBBox.y;

        bbox = cropRect;
    }

    // Data to add to the main screen (debugMat)
    // This is the movement raltive to the TL point of the
    // cv::line( debugMat, (bbox.tl()+bbox.br())/2,  (bbox.tl()+bbox.br())/2 + rotation.Point(40), cv::Scalar(0,0,255), 2, 4);
}

// Fixes a partial foreground image that may have been incorrectly identified
void RobotTracker::FixPartialForeground(cv::Mat &currFrame, cv::Mat &foreground, cv::Mat &new_fg_mask, cv::Rect &bestBBox)
{
    cv::Rect oldbbox = bbox;

    // the
    // Make sure old bbox width/height are at least as big as the previous one
    // Lets center the growth
    cv::Point deltaAdjust(0, 0);

    if (bbox.width < bestBBox.width)
    {
        deltaAdjust.x = (bestBBox.width - bbox.width) / 2;
        bbox.width = bestBBox.width;
    }
    if (bbox.height < bestBBox.height)
    {
        deltaAdjust.y = (bestBBox.height - bbox.height) / 2;
        bbox.height = bestBBox.height;
    }

    // Move bbox to new location
    bbox += delta - deltaAdjust;

    // Our old bbox was increased in size so that even though the new bestBBox area is much smaller, any single dimension is now equal to or
    // smaller then the old bbox as well.

    // Copy over foreground and mask
    cv::Mat temp_fg_image = foreground(bbox).clone();
    cv::Mat temp_fg_mask = new_fg_mask(bbox).clone();

    // Create a new mask that represents extra pixels being added
    // First create a blank slate
    cv::Mat delta_pixels_mask = cv::Mat::zeros(temp_fg_mask.size(), temp_fg_mask.type());

    // Next put in our old mask in the proper new location
    cv::Rect newPosMask(deltaAdjust.x, deltaAdjust.y, oldbbox.width, oldbbox.height);
    delta_pixels_mask(newPosMask) = finalMask;

    // Subtract the existing mask to get only delta pixels
    cv::subtract(delta_pixels_mask, temp_fg_mask, delta_pixels_mask);

    // Add these delta pixels to out temp_fgmask
    cv::bitwise_or(temp_fg_mask, delta_pixels_mask, temp_fg_mask);

    // Add these pixels to our foreground image
    cv::bitwise_or(temp_fg_image, currFrame(bbox), temp_fg_image, delta_pixels_mask);

    // Were all set, now save these
    fg_image = temp_fg_image;
    fg_mask = temp_fg_mask;
}

// FindNewPosAndRotUsingMatchTemplate
// currFrame - the current entire B&W frame
// foreground - the foreground extracted from the currFrame. Same size as currFrame
// fgFoundBBox: The rect region in foreground that contains the area we want to scane. It will be modified here to ensure it is bufferred, etc...
// newRot: contains the starting Rotation and result will be the new final rotation. sweep_rotation contains the incremental value
//
double RobotTracker::FindNewPosAndRotUsingMatchTemplate(cv::Mat &currFrame, cv::Mat &foreground, cv::Rect &fgFoundBBox, Vector2 &newRot)
{
    // We want to make sure the fgFoundBBox has the following:
    //  1) It is at least as big as our bbox
    //  2) We grow it in addition by the rotation growth
    //  3) Add a little buffer for some extra range
    cv::Rect newBBox;
    newBBox = fgFoundBBox;

    // Calculate the maximum rotated size that fgFoundBBox needs to be
    cv::Size newsize = RotateSize(bbox.size(), deltaAngleSweep);
    cv::Size bboxGrowth = newsize - bbox.size();

    // If the new foreground bbox is smaller than our old image, we need to grow it to allow scanning.
    // We don't know where the center is, thus we will grow it equally on all sides and hope any added buffer will
    // be sufficient.
    if (fgFoundBBox.width < bbox.width)
    {
        int delta = bbox.width - fgFoundBBox.width;
        fgFoundBBox.width = bbox.width;
        fgFoundBBox.x -= delta / 2;
    }

    if (fgFoundBBox.height < bbox.height)
    {
        int delta = bbox.height - fgFoundBBox.height;
        fgFoundBBox.height = bbox.height;
        fgFoundBBox.y -= delta / 2;
    }

    // Next we want to grow the scan area by the rotation growth and also add some buffer
    fgFoundBBox.x -= matchBufffer + bboxGrowth.width / 2;
    fgFoundBBox.y -= matchBufffer + bboxGrowth.height / 2;
    fgFoundBBox.width += 2 * matchBufffer + bboxGrowth.width;   // Increase it by the buffer
    fgFoundBBox.height += 2 * matchBufffer + bboxGrowth.height; // Increase it by the buffer

    // Translate bbox to make sure its inside
    fgFoundBBox = MoveRectToBeInside(fgFoundBBox, currFrame);

    cv::Mat matchingMat = currFrame(fgFoundBBox);

    // Try to find the position and rotation it moved to
    sweep_minVal = -1;
    sweep_maxVal = -1;
    sweep_rotation = -180;
    float currAngle = -1 * deltaAngleSweep;

    processes_done = 0;
    int processes_run = 0;

    float currAngleIncrement = 2.0 * deltaAngleSweep / ((float)numberOfThreads);

    for (float currAngle = -1 * deltaAngleSweep; currAngle < deltaAngleSweep; currAngle += currAngleIncrement)
    {
        float currAngleStop = currAngle + currAngleIncrement;

        if (!useMultithreading)
        {
            matchTemplateThread(fgFoundBBox, matchingMat, currAngle, currAngleStop);
        }
        else
        {

            ThreadPool::myThreads.enqueue([this, &fgFoundBBox, &matchingMat, currAngle, currAngleStop]
                              { this->matchTemplateThread(fgFoundBBox, matchingMat, currAngle, currAngleStop); });
            processes_run++;
        }
    }

    mutexResults.lock();
    while (processes_done < processes_run)
    {
        if (conditionVarResults.wait_for(mutexResults, std::chrono::seconds(2)) == std::cv_status::timeout)
        {
            markTime("Timeout occured in RobotTracker::FindNewPosAndRotUsingMatchTemplate !!! ");
        };
    }
    mutexResults.unlock();

    double newangle = atan2(newRot.y, newRot.x) - deg2rad(sweep_rotation); // sweep_rotation in anticlockwise
    delta_angle = sweep_rotation;

    newRot.x = cos(newangle);
    newRot.y = sin(newangle);

    delta_confidence = sweep_maxVal / sweep_maxVal_old;
    sweep_maxVal_old = sweep_maxVal;

    return sweep_maxVal;
}

// The matchTemplate thread that will be run multi-tasking
// matLocation: the abolute rect to position our matchingMat
// matchingMat to scan again
// currAngleStart = starting angle to rotate old foreground by
// currAngleStop = ending angle to rotate old foreground by
void RobotTracker::matchTemplateThread(const cv::Rect &matLocation, const cv::Mat &matchingMat, float currAngleStart, float currAngleStop)
{
    MultiThreadCleanup cleanup(processes_done, mutexResults, conditionVarResults);

    int random = rand() % 100;
    int i_debug = 0;
    bool debugDoDump = false;

    if (debug_DumpInfo && !debugImage.empty() && (debug_timeToSnapRotation > 0) && (currTimeSaved >= debug_timeToSnapRotation) && (currTimeSaved <= debug_timeToSnapRotation + debug_timeToSnapRotationduration))
    {
        debugDoDump = true;
    }

    cv::Mat rotated_fg_image;
    cv::Mat rotated_fg_mask;

    try
    {
        // markTime("Thread starting " + std::to_string(random) + ": ");
        for (float currAngle = currAngleStart; currAngle < currAngleStop; currAngle += deltaAngleStep)
        {
            // Rotate our previous fg image
            if (currAngle != 0)
            {
                cv::Point2f center_for_image(fg_image.cols / 2.0, fg_image.rows / 2.0);
                cv::Mat rotMat = cv::getRotationMatrix2D(center_for_image, currAngle, 1.0);

                // The rotational matrix doesn't actually work well: the corners that go to the left of the original BBox
                // are cut off. We need to move the cetner of the image back to the increased bbox center.

                cv::Size rotated_size = RotateSize(bbox.size(), currAngle);

                // chatGPT generated offset
                rotMat.at<double>(0, 2) += rotated_size.width / 2.0 - center_for_image.x;
                rotMat.at<double>(1, 2) += rotated_size.height / 2.0 - center_for_image.y;

                // Interpolation gives "fuzzy" corners which increases average error over non-rotated image, making the non-rotated
                // image more likelly to match. We thus have to use "INTER_NEAREST" (turn-off interpolation)
                cv::warpAffine(fg_image, rotated_fg_image, rotMat, rotated_size, cv::INTER_NEAREST);
                cv::warpAffine(fg_mask, rotated_fg_mask, rotMat, rotated_size, cv::INTER_NEAREST);
                // cv::threshold(rotated_fg_image, rotated_fg_mask, 2, 255, cv::THRESH_BINARY);
            }
            else
            {
                rotated_fg_image = fg_image;
                rotated_fg_mask = fg_mask;
            }

            // Sanity check on matchTemplate inputs
            if (matchingMat.cols < rotated_fg_image.cols ||
                matchingMat.rows < rotated_fg_image.rows ||
                rotated_fg_image.rows <= 1 ||
                rotated_fg_image.cols <= 1)
            {
                continue;
                // throw std::runtime_error("Invalid matching size vs rotated image."); //something went really wrong
            }

            // Do the matchTemplate
            cv::Mat matchResult;
            cv::matchTemplate(matchingMat, rotated_fg_image, matchResult, cv::TM_CCOEFF, rotated_fg_mask);

            // Correct results to favor expected location
            cv::Rect correctionROI = correctResultsForDistance(matchResult, matLocation, rotated_fg_image, GetRectCenter(bbox));

            // Find the location of the best match
            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(matchResult, &minVal, &maxVal, &minLoc, &maxLoc);

            if (debugDoDump)
            {
                // Draw the fg_image and fg_mask
                cv::Rect targetROI(86 * i_debug++, 20, rotated_fg_image.cols, rotated_fg_image.rows);

                // Add images if it doesn't exceed width
                if (targetROI.x + targetROI.width < debugImage.rows)
                {
                    cv::addWeighted(rotated_fg_mask, 0.5, debugImage(targetROI), 0.5, 0, debugImage(targetROI));
                    targetROI.y += 100;
                    cv::addWeighted(rotated_fg_image, 0.5, debugImage(targetROI), 0.5, 0, debugImage(targetROI));
                    int line = 0;

                    cv::rectangle(debugImage(targetROI), cv::Point(1, 1), cv::Point(targetROI.width - 2, targetROI.height - 2), cv::Scalar(255), 2);

                    printText("currAngle=" + std::to_string(int(currAngle * 100) / 100), debugImage, 200 + 40 * i_debug + 20 * line++, targetROI.x);
                    printText("maxVal=" + std::to_string(int(maxVal / 1000.0)), debugImage, 200 + 40 * i_debug + 20 * line++, targetROI.x);
                    printText("dim=()" + std::to_string(targetROI.width) + "," + std::to_string(targetROI.height) + ")", debugImage, 200 + 40 * i_debug + 20 * line++, targetROI.x);
                }
            }

            // If this is a better match than before, then remember it
            mutexResults.lock();
            if (maxVal >= sweep_maxVal)
            {
                sweep_minVal = minVal;
                sweep_maxVal = maxVal;
                sweep_minLoc = minLoc;
                sweep_maxLoc = maxLoc;
                sweep_rotation = currAngle;
                finalImage = rotated_fg_image;
                finalMask = rotated_fg_mask;
                finalBBox.width = finalMask.cols;
                finalBBox.height = finalMask.rows;
                finalBBox.y = maxLoc.y;
                finalBBox.x = maxLoc.x;
                matchResultSaved = matchResult;
                finalCorrectionROI = correctionROI;
                finalMatchingROI = matLocation;
            }
            mutexResults.unlock();
        }
    }
    catch (const std::exception &e)
    {
        if (debugDoDump)
        {
            printText("matchTemplateThread failed", debugImage, 20, 20);
        }
    }

    if (sweep_rotation < -170.0)
    {
        if (debugDoDump)
        {
            printText("matchTemplateThread failed", debugImage, 20, 20);
        }
    }
}

// Scales the results with a radial derating the further it is from the templ_center
cv::Rect RobotTracker::correctResultsForDistance(cv::Mat &results, const cv::Rect &scannedBox, const cv::Mat &templ, const cv::Point2f &templ_center)
{
    // The ROI from the derating ROI. Pre-populate width/height
    cv::Rect deratingROI(0, 0, results.cols, results.rows);

    if (!derateResultsByDistance)
    {
        return deratingROI;
    }

    // Get the expected location inside the results matrix
    // The top-left of the template is as follows:
    cv::Point2f ftempl_tl = templ_center;
    ftempl_tl.x -= templ.cols / 2.0;
    ftempl_tl.y -= templ.rows / 2.0;

    cv::Point2i temp_tl(ftempl_tl.x, ftempl_tl.y);

    cv::Point2i results_tl = temp_tl - scannedBox.tl();

    // Make sure the results make sense
    if (results_tl.x < 0 || results_tl.y < 0 || results_tl.x >= results.cols || results_tl.y >= results.rows)
    {
        return deratingROI;
    }

    // Now create the ROI of the derating image that would put the center location in the results anticipated location
    cv::Point2i tlOfDerating(distanceDerating.cols / 2, distanceDerating.rows / 2); // initialize to top-left-corner as the target
    tlOfDerating -= results_tl;                                                     // Move it so the center of derating mat is where results_tl tells us it should be
    deratingROI.x = tlOfDerating.x;
    deratingROI.y = tlOfDerating.y; // Create the Rect to extract image from

    // Now apply the derating
    cv::multiply(results, distanceDerating(deratingROI), results);

    return deratingROI;
}

void RobotTracker::SetRotation(double angleRad)
{
    rotation.x = cos(angleRad);
    rotation.y = sin(angleRad);
}
