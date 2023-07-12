#include "VisionPreprocessor.h"


VisionPreprocessor::VisionPreprocessor()
{
}



void VisionPreprocessor::Preprocess(cv::Mat& frame, cv::Mat& dst)
{
    // Define the four corners of the region of interest (ROI) in the input image
    cv::Point2f srcPoints[4] = {
        cv::Point2f(frame.cols * 0.25, frame.rows * 0.03),                   // Top-left corner
        cv::Point2f(frame.cols * 0.75, frame.rows * 0.03),          // Top-right corner
        cv::Point2f(frame.cols * 1.2, frame.rows), // Bottom-right corner
        cv::Point2f(-frame.cols * 0.3, frame.rows)           // Bottom-left corner
    };

    // Define the four corners of the bird's-eye view output image
    cv::Point2f dstPoints[4] = {
        cv::Point2f(0, 0),                             // Top-left corner
        cv::Point2f(WIDTH, 0),                   // Top-right corner
        cv::Point2f(WIDTH, HEIGHT), // Bottom-right corner
        cv::Point2f(0, HEIGHT)  // Bottom-left corner
    };

    // Compute the transformation matrix
    cv::Mat transformationMatrix = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // Apply the perspective transformation
    warpPerspective(dst, dst, transformationMatrix, cv::Size(WIDTH, HEIGHT));
}
