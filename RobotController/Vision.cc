#include "Vision.h"
#include "MathUtils.h"
#include "RobotStateParser.h"

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include "PathFinder.h"
#include <opencv2/flann.hpp>
#include "Globals.h"

Vision::Vision(ICameraReceiver &overheadCam, RobotTracker &robotTracker, RobotTracker &opponentTracker)
    : overheadCam(overheadCam),
      robotClassifier(robotTracker, opponentTracker),
      robotTracker(robotTracker),
      opponentTracker(opponentTracker)
{
}

bool Vision::areMatsEqual(const cv::Mat &mat1, const cv::Mat &mat2)
{
    if (mat1.size() != mat2.size() || mat1.type() != mat2.type())
    {
        // Mats have different sizes or types
        return false;
    }

    // Compute the absolute difference between the current frame and the previous frame
    cv::Mat diff;
    cv::absdiff(mat1, mat2, diff);

    // Convert the difference to grayscale
    cv::Mat grayDiff;
    cv::cvtColor(diff, grayDiff, cv::COLOR_BGR2GRAY);

    return cv::countNonZero(grayDiff) == 0;
}

void Vision::DetectRotation(cv::Mat& canny)
{
    const cv::Point2f robotPosition = GetRobotPosition();

    int CROP_RADIUS = 60;
    // crop us out
    cv::Mat cropped = canny(cv::Rect(robotPosition.x - CROP_RADIUS, robotPosition.y - CROP_RADIUS, CROP_RADIUS * 2, CROP_RADIUS * 2));

    cv::imshow("cropped", cropped);
    // get average angle of canny lines
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(cropped, lines, 1, CV_PI/180, 10, 3, 0);


    cv::Mat linesImage = cv::Mat::zeros(cropped.size(), CV_8UC3);
    ValueBin rotations(3);
    // show the vector of lines in the image
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        cv::line(linesImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,255), 1, cv::LINE_AA);

        double degrees = TO_DEG * atan2(l[3] - l[1], l[2] - l[0]);
        while (degrees < 0)
        {
            degrees += 180;
        }
        if (degrees == 0)
        {
            continue;
        }
        // take mode pi / 2
        double mode = fmod(degrees, 180);
    
        rotations.AddValue(mode);
    }


    // get mode of rotations
    double mode = rotations.GetModeValue();
    // convert to radians
    double radians = mode * TO_RAD;
    // draw arrow at middle of cropped
    cv::Point2f center = cv::Point2f(cropped.cols / 2, cropped.rows / 2);
    cv::Point2f end = center + cv::Point2f(cos(radians) * 100, sin(radians) * 100);
    cv::arrowedLine(linesImage, center, end, cv::Scalar(0, 255, 0), 3);


    cv::imshow("lines", linesImage);
}


/**
 * This is the 2d version of the pipeline
*/
bool Vision::runPipeline()
{
    overheadCam.getFrame(currFrame);

    birdsEyePreprocessor.Preprocess(currFrame, currFrame);

    // Skip the first frame or if the current frame is the same as the previous frame
    if (previousBirdsEye.empty() || areMatsEqual(currFrame, previousBirdsEye))
    {
        previousBirdsEye = currFrame.clone();
        return false;
    }

    drawingImage = currFrame.clone();

    // find the opponent
    locateRobots2d(currFrame, previousBirdsEye);
    previousBirdsEye = currFrame.clone();

    return true;
}

void Vision::locateRobots2d(cv::Mat& frame, cv::Mat& previousFrameL)
{
    const cv::Size BLUR_SIZE = cv::Size(14,14);

    const float MIN_AREA = pow(frame.cols * 0.037, 2);

    cv::Point2f center = cv::Point2f(0,0);

    // Compute the absolute difference between the current frame and the previous frame
    cv::Mat diff;
    cv::absdiff(previousFrameL, frame, diff);

    // Convert the difference to grayscale
    cv::Mat grayDiff;
    cv::cvtColor(diff, grayDiff, cv::COLOR_BGR2GRAY);

    // Convert the difference to a binary image with a certain threshold    
    cv::Mat thresholdImg;
    cv::threshold(grayDiff, thresholdImg, 25, 255, cv::THRESH_BINARY);

    // blurr and re-thresh to make it more leanient
    cv::blur(thresholdImg, thresholdImg, BLUR_SIZE);
    cv::threshold(thresholdImg, thresholdImg, 25, 255, cv::THRESH_BINARY);

    // cv::imshow("Motion", thresholdImg);

    // find big blobs in the image using a blob detector

    // iterate through every pixel in the image and find the largest blob
    std::vector<cv::Rect> potentialRobots = {};
    for (int y = 0; y < thresholdImg.rows; y += 10)
    {
        for (int x = 0; x < thresholdImg.cols; x += 10)
        {
            // if this pixel is white, then it is part of a blob
            if (thresholdImg.at<uchar>(y, x) == 255)
            {
                // flood fill the blob
                cv::Rect rect;
                // flood fill but don't change the image
                cv::floodFill(thresholdImg, cv::Point(x, y), cv::Scalar(100), &rect);

                // if the blob is larger than the previous largest blob, then update the largest blob
                if (rect.area() > MIN_AREA)
                {
                    // add the rect to the list of potential robots
                    potentialRobots.push_back(rect);
                }
            }
        }
    }

    // for each robot, find the EXACT center of the robot by counting the white pixels in the blob and averaging them
    std::vector<cv::Point2f> robotCenters;
    std::vector<MotionBlob> motionBlobs;
    for (const cv::Rect &rect : potentialRobots)
    {
        // find the average of all the white pixels in the blob
        int numWhitePixels = 0;
        cv::Point2f averageWhitePixel = cv::Point2f(0,0);
        for (int y = rect.y; y < rect.y + rect.height; y++)
        {
            for (int x = rect.x; x < rect.x + rect.width; x++)
            {
                // if this pixel is white, then add it to the average
                if (thresholdImg.at<uchar>(y,x) > 0)
                {
                    averageWhitePixel += cv::Point2f(x,y);
                    numWhitePixels++;
                }
            }
        }
        // divide by the number of white pixels to get the average
        averageWhitePixel /= numWhitePixels;

        // add the average to the list of robot centers
        motionBlobs.emplace_back(MotionBlob{rect, averageWhitePixel, &frame});
    }

    // classify the blobs and then update the trackers
    robotClassifier.Update(motionBlobs, frame, thresholdImg);
}

cv::Point2f Vision::GetRobotPosition()
{
    return robotTracker.getPosition();
}

double Vision::GetRobotAngle()
{
    return angle;
}

cv::Point2f Vision::GetOpponentPosition()
{
    return opponentTracker.getPosition();
}

double Vision::GetOpponentAngle()
{
    return opponentTracker.getAngle();
}

const cv::Mat& Vision::GetBirdsEyeImage()
{
    return previousBirdsEye;
}