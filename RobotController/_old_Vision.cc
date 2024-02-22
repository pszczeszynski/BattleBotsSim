#include "Vision.h"
#include "MathUtils.h"
#include "RobotStateParser.h"

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include "PathFinder.h"
#include <opencv2/flann.hpp>
#include "Globals.h"
#include "RobotConfig.h"
#include "UIWidgets/RobotControllerGUI.h"
#include <winuser.h>

Vision::Vision(ICameraReceiver &overheadCam)
    : overheadCam(overheadCam),
      _lastDrawingImage(cv::Mat::zeros(WIDTH, HEIGHT, CV_8UC3))
{
    // create the processing thread in a loop
    processingThread = std::thread([&]()
                                   {
        // holds the data of the current frame
        cv::Mat currFrame;
        long frame_id = -1;

        while (true)
        {
            // get the current frame from the camera
            // This guarantees a frame is gotten unless error occured (e.g. it waits until frame is available)
            frame_id = overheadCam.GetFrame(currFrame, frame_id);

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
    // No longer required done in captureVideo
    // birdsEyePreprocessor.Preprocess(currFrame, currFrame);

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

/**
 * Locates the robots in the frame
*/
VisionClassification Vision::LocateRobots2d(cv::Mat& frame, cv::Mat& previousFrameL)
{
    static ImageWidget motionImageWidget{"Motion", true};

    // hyperparameters
    const cv::Size BLUR_SIZE = cv::Size(14,14);
    const float MIN_AREA = pow(min(MIN_OPPONENT_BLOB_SIZE, MIN_ROBOT_BLOB_SIZE), 2);
    const float MAX_AREA = pow(max(MAX_OPPONENT_BLOB_SIZE, MAX_ROBOT_BLOB_SIZE), 2);
    const int BLOB_SEARCH_SIZE = 10;

    // Compute the absolute difference between the current frame and the previous frame
    cv::Mat diff;
    cv::absdiff(previousFrameL, frame, diff);

    // Convert the difference to grayscale
    cv::Mat grayDiff;

    if (diff.channels() == 1) {
        grayDiff = diff;
    }
    else if (diff.channels() == 3) {
        cv::cvtColor(diff, grayDiff, cv::COLOR_BGR2GRAY);
    }
    else if (diff.channels() == 4)
    {
        cv::cvtColor(diff, grayDiff, cv::COLOR_BGRA2GRAY);
    }


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

    // classify the blobs and save them for later
    return robotClassifier.ClassifyBlobs(motionBlobs, frame, thresholdImg);
}

VisionPreprocessor& Vision::GetPreprocessor()
{
    return birdsEyePreprocessor;
}