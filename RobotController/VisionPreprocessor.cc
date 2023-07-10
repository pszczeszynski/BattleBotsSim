#include "VisionPreprocessor.h"


VisionPreprocessor::VisionPreprocessor()
{
}

void VisionPreprocessor::Preprocess(cv::Mat& frame, cv::Mat& dst)
{
    // scale to 1280 by 720
    cv::resize(frame, dst, cv::Size(WIDTH, HEIGHT));

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
    warpPerspective(dst, dst, transformationMatrix, cv::Size(outputWidth, outputHeight));
}
