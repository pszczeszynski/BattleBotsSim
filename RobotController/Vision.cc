#include "Vision.h"
#include "MathUtils.h"
#include "RobotStateParser.h"

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include "PathFinder.h"
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/flann.hpp>

#define WIDTH 1280
#define HEIGHT 720

Vision::Vision(ICameraReceiver &overheadCam)
    : overheadCam(overheadCam)
{
    // add 2 robot trackers
    robotTrackers.push_back(RobotTracker(cv::Point2f(0,0)));
    robotTrackers.push_back(RobotTracker(cv::Point2f(10000, 10000)));
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

void Vision::DrawRobots()
{
    cv::Mat frameWithCircles = currFrame.clone();

    const cv::Point2f robotPosition = GetRobotPosition();
    const cv::Point2f opponentPosition = GetOpponentPosition();
    const double robotAngle = angle;//GetRobotAngle();
    const double opponentAngle = opponent_angle;//GetOpponentAngle();

    // draw lines starting at the center of each robot showing their angles
    const double RADIUS = 30;

    cv::circle(frameWithCircles, robotPosition, RADIUS, cv::Scalar(255,0,0), 4);
    cv::circle(frameWithCircles, opponentPosition, RADIUS, cv::Scalar(0,0,255), 4);

    cv::line(frameWithCircles, robotPosition, robotPosition + cv::Point2f(cos(robotAngle) * RADIUS, sin(robotAngle) * RADIUS), cv::Scalar(255,0,0), 4);
    cv::line(frameWithCircles, opponentPosition, opponentPosition + cv::Point2f(cos(opponentAngle) * RADIUS, sin(opponentAngle) * RADIUS), cv::Scalar(0,0,255), 4);

    cv::imshow("circles", frameWithCircles);
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
void Vision::runPipeline()
{
    overheadCam.getFrame(currFrame);

    // scale to 1280 by 720
    cv::resize(currFrame, currFrame, cv::Size(WIDTH, HEIGHT));

    convertToBirdsEyeView2d(currFrame, currFrame);

    // // Skip the first frame or if the current frame is the same as the previous frame
    // if (previousBirdsEye.empty() || areMatsEqual(currFrame, previousBirdsEye))
    // {
    //     previousBirdsEye = currFrame.clone();
    //     return;
    // }

    // // find the opponent
    // locateRobots2d(currFrame, previousBirdsEye);
    previousBirdsEye = currFrame.clone();
}

void Vision::locateRobots2d(cv::Mat& frame, cv::Mat& previousFrameL)
{
    const cv::Size BLUR_SIZE = cv::Size(14,14);

    const float MIN_AREA = 1200;

    cv::Point2f center = cv::Point2f(0,0);

    // Compute the absolute difference between the current frame and the previous frame
    cv::Mat diff;
    cv::absdiff(previousFrameL, frame, diff);

    // Convert the difference to grayscale
    cv::Mat grayDiff;
    cv::cvtColor(diff, grayDiff, cv::COLOR_BGR2GRAY);

    // Convert the difference to a binary image with a certain threshold    
    cv::Mat thresholdImg;
    cv::threshold(grayDiff, thresholdImg, 50, 255, cv::THRESH_BINARY);

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

    // update the robot trackers
    updateRobotTrackers(motionBlobs, frame);
}

cv::Point2f Vision::GetRobotPosition()
{
    return position;//robotTrackers[0].getPosition();
}

double Vision::GetRobotAngle()
{
    return angle;//robotTrackers[0].getAngle();
}

cv::Point2f Vision::GetOpponentPosition()
{
    return opponent_position;//robotTrackers[1].getPosition();
}

double Vision::GetOpponentAngle()
{
    return opponent_angle;//robotTrackers[1].getAngle();
}

/**
 * Assigns existing trackers to the closest blobs
*/
void Vision::updateRobotTrackers(std::vector<MotionBlob>& centers, cv::Mat& frame)
{
    // minimize cost of updating
    if (centers.empty()) return;

    double COST_THRESHOLD = 0.15; // above this cost means we're not updating
    std::vector<bool> updatedTrackers = {};
    // fill updatedTrackers with false
    for (int i = 0; i < robotTrackers.size(); i++) updatedTrackers.push_back(false);

    if (centers.size() == 1)
    {
        double cost1 = robotTrackers[0].getCostOfUpdating(centers[0]);
        double cost2 = robotTrackers[1].getCostOfUpdating(centers[0]);
        // std::cout << "cost1: " << cost1 << std::endl;

        // assume the robot is us if we're moving
        if (cost1 < COST_THRESHOLD)
        {
            // if (cost1 < cost2)
            // {
                robotTrackers[0].update(centers[0], frame);
                updatedTrackers[0] = true;
            // }
            // else
            // {
            //     robotTrackers[1].update(centers[0], frame);
            //     updatedTrackers[1] = true;
            // }
        }
    }
    else
    {
        double cost00 = robotTrackers[0].getCostOfUpdating(centers[0]);
        double cost01 = robotTrackers[0].getCostOfUpdating(centers[1]);
        double cost10 = robotTrackers[1].getCostOfUpdating(centers[0]);
        double cost11 = robotTrackers[1].getCostOfUpdating(centers[1]);

        if (cost00 + cost11 < cost01 + cost10)
        {
            if (cost00 < COST_THRESHOLD)
            {
                robotTrackers[0].update(centers[0], frame);
                updatedTrackers[0] = true;
            }

            if (cost11 < COST_THRESHOLD)
            {
                robotTrackers[1].update(centers[1], frame);
                updatedTrackers[1] = true;
            }
        }
        else
        {
            if (cost01 < COST_THRESHOLD)
            {
                robotTrackers[0].update(centers[1], frame);
                updatedTrackers[0] = true;
            }

            if (cost10 < COST_THRESHOLD)
            {
                robotTrackers[1].update(centers[0], frame);
                updatedTrackers[1] = true;
            }
        }
    }

    // for each tracker that wasn't updated, mark it as invalid
    for (int i = 0; i < robotTrackers.size(); i++)
    {
        if (!updatedTrackers[i])
        {
            robotTrackers[i].invalidate();
        }
    }
}

void Vision::convertToBirdsEyeView2d(cv::Mat &frame, cv::Mat &dst)
{
    // Define the four corners of the region of interest (ROI) in the input image
    cv::Point2f srcPoints[4] = {
        cv::Point2f(0, 0),                   // Top-left corner
        cv::Point2f(frame.cols, 0),          // Top-right corner
        cv::Point2f(frame.cols, frame.rows), // Bottom-right corner
        cv::Point2f(0, frame.rows)           // Bottom-left corner
    };

    // Define the dimensions of the bird's-eye view output image
    int outputWidth = frame.cols;
    int outputHeight = frame.rows;

    // Define the four corners of the bird's-eye view output image
    cv::Point2f dstPoints[4] = {
        cv::Point2f(0, 0),                             // Top-left corner
        cv::Point2f(outputWidth, 0),                   // Top-right corner
        cv::Point2f(outputWidth * 0.65, outputHeight), // Bottom-right corner
        cv::Point2f(outputWidth * 0.35, outputHeight)  // Bottom-left corner
    };

    // Compute the transformation matrix
    cv::Mat transformationMatrix = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // Apply the perspective transformation
    warpPerspective(frame, dst, transformationMatrix, cv::Size(outputWidth, outputHeight));
}


const cv::Mat& Vision::GetBirdsEyeImage()
{
    return previousBirdsEye;
}