#include "Vision.h"
#include "MathUtils.h"
#include "RobotStateParser.h"

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include "PathFinder.h"
#include <opencv2/flann.hpp>
#include "Globals.h"

Vision::Vision(ICameraReceiver &overheadCam)
    : overheadCam(overheadCam)
{
    // create the processing thread in a loop
    processingThread = std::thread([&]()
                                   {
        // holds the data of the current frame
        cv::Mat currFrame;
        while (true)
        {
            // get the current frame from the camera
            bool hadFrame = overheadCam.GetFrame(currFrame);
            // if we didn't get a frame, return no classification
            if (!hadFrame)
            {
                // sleep for 1ms
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue; // try to get a frame again
            }

            // run the pipeline
            VisionClassification classification = RunPipeline(currFrame);

            // save the classification
            _classificationMutex.lock();
            _classification = classification;
            currFrame.copyTo(_lastDrawingImage);
            _classificationMutex.unlock();
        } });
}

/**
 * Consumes the latest classification
 * Will reset the stored classification to no classification
 * @return the latest classification
*/
VisionClassification Vision::ConsumeLatestClassification(cv::Mat& outDrawingImage)
{
    // lock the mutex
    _classificationMutex.lock();
    // get the classification
    VisionClassification classification = _classification;
    // copy the last drawing image
    _lastDrawingImage.copyTo(outDrawingImage);
    // reset the classification
    _classification = VisionClassification();
    // unlock the mutex
    _classificationMutex.unlock();

    // return the classification
    return classification;
}

/**
 * This is the 2d version of the pipeline
*/
VisionClassification Vision::RunPipeline(cv::Mat& currFrame)
{
    VisionClassification ret;

    ret.SetHadNewImage();

    // preprocess the frame to get the birds eye view
    birdsEyePreprocessor.Preprocess(currFrame, currFrame);

    // if we don't have a previous frame
    if (previousBirdsEye.empty())
    {
        // set the previous frame to the current frame
        previousBirdsEye = currFrame.clone();
        _prevFrameTimer.markStart();
        // set the flag that we had a new image
        ret.SetHadNewImage();
        // return no classification since we don't have a previous frame
        return ret;
    }

    // find the opponent
    ret = LocateRobots2d(currFrame, previousBirdsEye);
    if (ret.GetRobotBlob() != nullptr || ret.GetOpponentBlob() != nullptr || _prevFrameTimer.getElapsedTime() > 0.05)
    {
        // save the current frame as the previous frame
        previousBirdsEye = currFrame.clone();
        _prevFrameTimer.markStart();
    }
    // set the flag that we had a new image
    ret.SetHadNewImage();

    // return the classification
    return ret;
}

VisionClassification Vision::LocateRobots2d(cv::Mat& frame, cv::Mat& previousFrameL)
{
    const cv::Size BLUR_SIZE = cv::Size(14,14);

    const float MIN_AREA = pow(frame.cols * 0.07, 2);

    cv::Point2f center = cv::Point2f(0,0);

    // Compute the absolute difference between the current frame and the previous frame
    cv::Mat diff;
    cv::absdiff(previousFrameL, frame, diff);

    // Convert the difference to grayscale
    cv::Mat grayDiff;
    cv::cvtColor(diff, grayDiff, cv::COLOR_BGR2GRAY);

    const int LOW_THRESHOLD = 14;

    // Convert the difference to a binary image with a certain threshold    
    cv::Mat thresholdImg;
    cv::threshold(grayDiff, thresholdImg, LOW_THRESHOLD, 255, cv::THRESH_BINARY);

    // blurr and re-thresh to make it more leanient
    cv::blur(thresholdImg, thresholdImg, BLUR_SIZE);
    cv::threshold(thresholdImg, thresholdImg, 15, 255, cv::THRESH_BINARY);
    

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
    std::vector<MotionBlob> motionBlobs = {};

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

    cv::imshow("thresh", thresholdImg);
    cv::waitKey(1);

    // classify the blobs and save them for later
    return robotClassifier.ClassifyBlobs(motionBlobs, frame, thresholdImg);
}

VisionPreprocessor& Vision::GetPreprocessor()
{
    return birdsEyePreprocessor;
}