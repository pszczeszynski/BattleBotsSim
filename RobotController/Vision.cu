#include "Vision.h"
#include "MathUtils.h"
#include "RobotStateParser.h"

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>

Vision::Vision(CameraReceiver &overheadCam)
    : overheadCam(overheadCam),
      opticalFlow(),
      opponentOpticalFlow()
{
    // add 2 robot trackers
    robotTrackers.push_back(RobotTracker(cv::Point2f(0,0)));
    robotTrackers.push_back(RobotTracker(cv::Point2f(10000, 10000)));

}

void Vision::performOpticalFlow()
{
    cv::Mat frame = overheadCam.getFrame();
    cv::Mat drawingImage = frame.clone();

    // upload to gpu + convert to grayscale
    cv::cuda::GpuMat gpuFrame;
    cv::cuda::GpuMat gpuGrayFrame;
    gpuFrame.upload(frame);
    cv::cuda::cvtColor(gpuFrame, gpuGrayFrame, cv::COLOR_BGR2GRAY);


    if (!isInitialized)
    {
        opticalFlow.InitializeMotionDetection(gpuGrayFrame);
        isInitialized = true;
    }
    else
    {
        opticalFlow.PerformMotionDetection(gpuGrayFrame, drawingImage);
        cv::imshow("optical flow", drawingImage);
    }
}

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
    // get the frame from the camera
    cv::Mat frame = overheadCam.getFrame().clone();


    // Skip the first frame or if the current frame is the same as the previous frame
    if (previousFrame.empty() || areMatsEqual(frame, previousFrame))
    {
        previousFrame = frame;
        return;
    }

    // find the opponent
    cv::Point2f opponent_new = findOpponent(frame, previousFrame);

    // crop the image around the opponent
    unsigned int cropSize = 50;
    cv::Rect opponentRect = cv::Rect(opponent_new.x - cropSize / 2, opponent_new.y - cropSize / 2, cropSize, cropSize);
    // make sure the crop rectangle doesn't go out of bounds
    opponentRect.x = std::max(0, opponentRect.x);
    opponentRect.y = std::max(0, opponentRect.y);
    opponentRect.width = std::min(opponentRect.width, frame.cols - opponentRect.x);
    opponentRect.height = std::min(opponentRect.height, frame.rows - opponentRect.y);

    cv::Mat croppedDrawingImage = frame(opponentRect).clone();
    std::cout << "about to crop. Crop size: " + std::to_string(cropSize) << std::endl;

    cv::cuda::GpuMat gpuFrame;
    cv::cuda::GpuMat gpuGrayFrame;
    gpuFrame.upload(croppedDrawingImage);
    cv::cuda::cvtColor(gpuFrame, gpuGrayFrame, cv::COLOR_BGR2GRAY);
    std::cout << "cropped" << std::endl;

    // if (!isInitialized)
    // {
    //     std::cout << "about to init" << std::endl;
    //     opponentOpticalFlow.InitializeMotionDetection(gpuGrayFrame);
    //     std::cout << "init done" << std::endl;
    //     isInitialized = true;
    // }
    // else
    // {
    //     std::cout << "about to perform motion detection" << std::endl;
    //     opponentOpticalFlow.PerformMotionDetection(gpuGrayFrame, croppedDrawingImage, true, true, false);
    //     std::cout << "motion detection done" << std::endl;
    //     // cv::imshow("optical flow", croppedDrawingImage);
    // }

    cv::waitKey(1);


    // save the current frame
    previousFrame = frame;
}

cv::Size blurSize = cv::Size(14,14);

cv::Point2f Vision::findOpponent(cv::Mat& frame, cv::Mat& previousFrame)
{
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
    cv::blur(thresholdImg, thresholdImg, blurSize);
    cv::threshold(thresholdImg, thresholdImg, 25, 255, cv::THRESH_BINARY);
    cv::imshow("threshold", thresholdImg);

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

    updateRobotTrackers(motionBlobs, frame);

    // clone the frame so we can draw on it
    cv::Mat frameWithCircles = frame.clone();

    cv::Point2f pos1 = robotTrackers[0].getPosition();
    double ang1 = robotTrackers[0].getAngle();
    cv::Point2f pos2 = robotTrackers[1].getPosition();
    double ang2 = robotTrackers[1].getAngle();

    cv::circle(frameWithCircles, pos1, 30, cv::Scalar(255,0,0), 4);
    cv::circle(frameWithCircles, pos2, 30, cv::Scalar(0,0,255), 4);

    // ang1 = angle_wrap(opponentOpticalFlow.GetRotation());

    // std::cout << "velocity: " << robotTrackers[0].getVelocity() << std::endl;
    // if (cv::norm(robotTrackers[0].getVelocity()) > 200)
    // {
    //     double velocity_angle = atan2(robotTrackers[0].getVelocity().y, robotTrackers[0].getVelocity().x);
    //     if (abs(angle_wrap(velocity_angle - ang1)) > M_PI / 2)
    //     {
    //         velocity_angle += M_PI;
    //     }
    //     velocity_angle = angle_wrap(velocity_angle);

    //     if (abs(angle_wrap(velocity_angle - ang1)) > M_PI / 4)
    //     {
    //         ang1 = velocity_angle;
    //         opponentOpticalFlow.SetRotation(velocity_angle);
    //     }
    // }

    // draw lines starting at the center of each robot showing their angles
    const double RADIUS = 30;
    cv::line(frameWithCircles, pos1, pos1 + cv::Point2f(cos(ang1) * RADIUS, sin(ang1) * RADIUS), cv::Scalar(255,0,0), 4);
    cv::line(frameWithCircles, pos2, pos2 + cv::Point2f(cos(ang2) * RADIUS, sin(ang2) * RADIUS), cv::Scalar(0,0,255), 4);

    // scale up th e
    cv::imshow("keypoints", frameWithCircles);

    return pos1;
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
        std::cout << "cost1: " << cost1 << std::endl;

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