#ifndef CORNERTRACKER_H
#define CORNERTRACKER_H

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/cudaoptflow.hpp"
#include "opencv2/cudaimgproc.hpp"
#include <opencv2/core/utility.hpp>
#include "Corner.h"

class CornerTracker
{
private:
    // CONSTANTS
    const int MAX_CORNERS;
    // Multiply this by the best corner's quality gives the min quality we accept
    const float MIN_CORNER_QUALITY_PERCENTAGE;
    // min distance between corners
    const int MIN_CORNER_DIST;
    // size of each corner's box tracker
    const int CORNER_SIZE;
    // Used to detect corners, not sure what it is
    const float K_VALUE;
    // Used to remove points too close to the edge of the image.
    const int BORDER_SIZE;
    int imageWidth;
    int imageHeight;

    // for motion detection
    bool isInitialized = false;

    cv::cuda::GpuMat lastFrame;

    // CUDA
    cv::Ptr<cv::cuda::CornersDetector> detector;
    cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> tracker;
    std::vector<cv::Point2f> GpuPointMatToVector(cv::cuda::GpuMat&);

    void FindCrazyTrackers();

public:
    // tracking point information
    // std::vector<uchar> status;
    // std::vector<float> err;

    // where the corners currently are
    std::vector<cv::Point2f> pf;
    // where the tracker started from
    std::vector<cv::Point2f> pi;
    // tracker positions on the last frame.
    std::vector<cv::Point2f> pl;

    std::vector<Corner> corners;

    cv::cuda::GpuMat pl_GPU;
    cv::cuda::GpuMat pf_GPU;

    void PerformMotionDetection(cv::cuda::GpuMat&, cv::Mat&);
    void ChooseNewTrackingPoints(cv::cuda::GpuMat&);
    void CalculateTrackerBins(double shiftAngle = 0, double shiftScale = 0);

    double AverageTranslationMagnitude();
    double MedianTranslationMagnitude();
    double LargestBoundingBoxSide();

    void DrawTrackers(cv::Mat&, cv::Scalar);

    CornerTracker();

    // Delete copy assignment since we
    CornerTracker operator=(const CornerTracker &rhs) = delete;
};

#endif //CORNERTRACKER_H
