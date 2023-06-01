
#include "CornerTracker.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/video/video.hpp>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <map>
#include "TrackingUtils.h"
#include "Timer.h"
#include "opencv2/cudaoptflow.hpp"
#include "opencv2/cudaimgproc.hpp"

using namespace std;
using namespace cv;

#define PI 3.14159265358979323

CornerTracker::CornerTracker()
    : MAX_CORNERS(100),
      MIN_CORNER_QUALITY_PERCENTAGE(0.001),
      MIN_CORNER_DIST(7),
      CORNER_SIZE(7),
      K_VALUE(0.04),
      BORDER_SIZE(16)
{
    detector = cuda::createGoodFeaturesToTrackDetector(CV_8UC1, MAX_CORNERS, MIN_CORNER_QUALITY_PERCENTAGE, MIN_CORNER_DIST, CORNER_SIZE, false, K_VALUE);
    tracker = cuda::SparsePyrLKOpticalFlow::create(cv::Size(15, 15), 3, 10);
}

void CornerTracker::ChooseNewTrackingPoints(cv::cuda::GpuMat &frame)
{
    cv::cuda::GpuMat newPointsGPU;
    detector->detect(frame, newPointsGPU);
    std::vector<cv::Point2f> newPi = GpuPointMatToVector(newPointsGPU);

    pl_GPU = newPointsGPU.clone();
    lastFrame = frame.clone();

    pi = newPi;
    pl = pi;
    // set pf = pi since we haven't tracked this frame yet
    pf = pi;
    imageWidth = frame.size().width;
    imageHeight = frame.size().height;
}

double CornerTracker::LargestBoundingBoxSide()
{
    // find the largest dimension of the bounding box for the points
    int minX = 0;
    int maxX = 0;
    int minY = 0;
    int maxY = 0;
    for (uint i = 0; i < pl.size(); i++)
    {
        if (pl[i].x < minX)
        {
            minX = pl[i].x;
        }
        if (pl[i].x > maxX)
        {
            maxX = pl[i].x;
        }
        if (pl[i].y < minY)
        {
            minY = pl[i].y;
        }
        if (pl[i].y > maxY)
        {
            maxY = pl[i].y;
        }
    }
    return max(maxX - minX, maxY - minY);
}

std::vector<cv::Point2f> CornerTracker::GpuPointMatToVector(cv::cuda::GpuMat &mat)
{
    std::vector<cv::Point2f> ret = {};
    cv::Mat cpuMat(mat);
    for (int i = 0; i < cpuMat.size().width; i++)
    {
        cv::Vec2f thisPoint = cpuMat.at<cv::Vec2f>(0, i);
        ret.push_back(cv::Point2f(thisPoint[0], thisPoint[1]));
    }

    return ret;
}

void CornerTracker::FindCrazyTrackers()
{
    const int MAX_MOVEMENT_PER_FRAME = 100;
    const int MAX_MOVEMENT_ABOVE_MEDIAN = 30;
    double medianTranslation = MedianTranslationMagnitude();

    int numCrazy = 0;
    for (uint i = 0; i < corners.size(); i++)
    {
        double magnitude = corners[i].GetTravelDistLastFrame(false);

        if (magnitude > MAX_MOVEMENT_ABOVE_MEDIAN + medianTranslation || magnitude > MAX_MOVEMENT_PER_FRAME ||
            corners[i].pf.x < BORDER_SIZE || corners[i].pf.x > imageWidth - BORDER_SIZE ||
            corners[i].pf.y < BORDER_SIZE || corners[i].pf.y > imageHeight - BORDER_SIZE)
        {
            corners[i].isCrazy = true;
            numCrazy ++;
        }
    }

    if (numCrazy > 0)
    {
        std::cout << "FOUND " << numCrazy << " crazy corners!" << std::endl;
    }
}

double CornerTracker::AverageTranslationMagnitude()
{
    double magnitudeSum = 0;
    for (uint i = 0; i < corners.size(); i++)
    {
        magnitudeSum += corners[i].GetTravelDistLastFrame(false);
    }

    return magnitudeSum / pl.size();
}

double CornerTracker::MedianTranslationMagnitude()
{
    std::vector<double> magnitudes = {};
    for (uint i = 0; i < corners.size(); i++)
    {
        magnitudes.push_back(corners[i].GetTravelDistLastFrame(false));
    }

    std::sort(magnitudes.begin(), magnitudes.end());

    return magnitudes[magnitudes.size() / 2];
}

void CornerTracker::PerformMotionDetection(cv::cuda::GpuMat &frame, cv::Mat& drawingImage)
{
    if (pf.size() > 0)
    {
        pl = pf;
    }

    cv::cuda::GpuMat status_GPU;
    cv::cuda::GpuMat err_GPU;

    // equalizeHist normalizes the histogram of the image.
    // this fixes problems where exposure/brightness changes in the image would cause massive tracking innacuracies
    cv::cuda::equalizeHist(frame, frame);

    tracker->calc(lastFrame, frame, pl_GPU, pf_GPU, status_GPU, err_GPU);
    pf = GpuPointMatToVector(pf_GPU);

    std::cout << "pi.size(): " << pi.size() << " pl.size(): " << pl.size() << " pf.size(): " << pf.size() << std::endl;

    // this is a deep copy, see if we can double buffer
    lastFrame = frame.clone();
    pl_GPU = pf_GPU.clone();


    corners = {};
    for (uint i = 0; i < pl.size(); i++)
    {
        corners.push_back(Corner(pi[i], pl[i], pf[i]));
    }

    FindCrazyTrackers();
}

void CornerTracker::DrawTrackers(cv::Mat &drawingImage, cv::Scalar color)
{
    for (uint i = 0; i < pi.size(); i++)
    {
        cv::line(drawingImage, pf[i], pi[i], color, 2);
    }
}
