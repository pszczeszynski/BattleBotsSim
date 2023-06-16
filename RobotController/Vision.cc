#include "Vision.h"
#include "MathUtils.h"
#include "RobotStateParser.h"

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include "PathFinder.h"


Vision::Vision(CameraReceiver &overheadCam)
    : overheadCam(overheadCam),
      opticalFlow(),
      opponentOpticalFlow()
{
    // add 2 robot trackers
    robotTrackers.push_back(RobotTracker(cv::Point2f(0,0)));
    robotTrackers.push_back(RobotTracker(cv::Point2f(10000, 10000)));
}

// void Vision::performOpticalFlow()
// {
//     cv::Mat frame = overheadCam.getFrame();
//     cv::Mat drawingImage = frame.clone();

//     // upload to gpu + convert to grayscale
//     cv::cuda::GpuMat gpuFrame(frame);
//     cv::cuda::GpuMat gpuGrayFrame;

//     cv::cuda::cvtColor(gpuFrame, gpuGrayFrame, cv::COLOR_BGR2GRAY);


//     if (!isInitialized)
//     {
//         opticalFlow.InitializeMotionDetection(gpuGrayFrame);
//         isInitialized = true;
//     }
//     else
//     {
//         opticalFlow.PerformMotionDetection(gpuGrayFrame, drawingImage);
//         cv::imshow("optical flow", drawingImage);
//     }
// }

bool areMatsEqual(const cv::Mat &mat1, const cv::Mat &mat2)
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

void Vision::runPipeline()
{
    Clock c2;
    c2.markStart();
    // get the frame from the camera
    overheadCam.getFrame(currFrame);
    convertToBirdsEyeView(currFrame, currFrame);
    // scale to 1280 by 720
    cv::resize(currFrame, currFrame, cv::Size(1280, 720));

    // // Skip the first frame or if the current frame is the same as the previous frame
    // if (previousFrame.empty() || areMatsEqual(currFrame, previousFrame))
    // {
    //     previousFrame = currFrame.clone();
    //     return;
    // }
    robotTrackers[0].angle = angle; // TODO: remove this
    robotTrackers[0].position = position;
    robotTrackers[1].angle = opponent_angle;
    robotTrackers[1].position = opponent_position;

    // // find the opponent
    // locateRobots(currFrame, previousFrame);

    // save the current frame
    previousFrame = currFrame.clone();

    std::cout << "total time runPipeline" << c2.getElapsedTime() << std::endl;
}

const cv::Mat& Vision::GetBirdsEyeImage()
{
    return previousFrame;
}

void Vision::locateRobots(cv::Mat& frame, cv::Mat& previousFrame)
{
    const cv::Size BLUR_SIZE = cv::Size(14,14);

    const float MIN_AREA = 1200;

    cv::Point2f center = cv::Point2f(0,0);

    // Compute the absolute difference between the current frame and the previous frame
    cv::Mat diff;
    cv::absdiff(previousFrame, frame, diff);

    // Convert the difference to grayscale
    cv::Mat grayDiff;
    cv::cvtColor(diff, grayDiff, cv::COLOR_BGR2GRAY);

    // Convert the difference to a binary image with a certain threshold    
    cv::Mat thresholdImg;
    cv::threshold(grayDiff, thresholdImg, 50, 255, cv::THRESH_BINARY);

    // blurr and re-thresh to make it more leanient
    cv::blur(thresholdImg, thresholdImg, BLUR_SIZE);
    cv::threshold(thresholdImg, thresholdImg, 25, 255, cv::THRESH_BINARY);

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
    return robotTrackers[0].getPosition();
}

double Vision::GetRobotAngle()
{
    return robotTrackers[0].getAngle();
}

cv::Point2f Vision::GetOpponentPosition()
{
    return robotTrackers[1].getPosition();
}

double Vision::GetOpponentAngle()
{
    return robotTrackers[1].getAngle();
}

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
            robotTrackers[0].update(centers[0], frame);
            updatedTrackers[0] = true;
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

void Vision::convertToBirdsEyeView(cv::Mat& frame, cv::Mat& dst)
{
    // Define the four corners of the region of interest (ROI) in the input image
    cv::Point2f srcPoints[4] = {
        cv::Point2f(0, 0),    // Top-left corner
        cv::Point2f(frame.cols, 0),    // Top-right corner
        cv::Point2f(frame.cols, frame.rows),    // Bottom-right corner
        cv::Point2f(0, frame.rows)     // Bottom-left corner
    };

    // Define the dimensions of the bird's-eye view output image
    int outputWidth = frame.cols;
    int outputHeight = frame.rows;

    // Define the four corners of the bird's-eye view output image
    cv::Point2f dstPoints[4] = {
        cv::Point2f(0, 0),                      // Top-left corner
        cv::Point2f(outputWidth, 0),        // Top-right corner
        cv::Point2f(outputWidth * 0.65, outputHeight),  // Bottom-right corner
        cv::Point2f(outputWidth * 0.35, outputHeight)        // Bottom-left corner
    };

    // Compute the transformation matrix
    cv::Mat transformationMatrix = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // Apply the perspective transformation
    warpPerspective(frame, dst, transformationMatrix, cv::Size(outputWidth, outputHeight));
}
