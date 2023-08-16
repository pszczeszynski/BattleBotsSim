#include "VisionPreprocessor.h"
#include "RobotConfig.h"

const int CLOSE = 20;

VisionPreprocessor::VisionPreprocessor()
{
    // Define the four corners of the bird's-eye view output image
    dstPoints[0] = cv::Point2f(0, 0);          // Top-left corner
    dstPoints[1] = cv::Point2f(WIDTH, 0);      // Top-right corner
    dstPoints[2] = cv::Point2f(WIDTH, HEIGHT); // Bottom-right corner
    dstPoints[3] = cv::Point2f(0, HEIGHT);     // Bottom-left corner
}

void VisionPreprocessor::Preprocess(cv::Mat &frame, cv::Mat &dst)
{
    cv::Point2f srcPoints[4];
    // Define the four corners of the bird's-eye view input image
    srcPoints[0] = cv::Point2f(preprocess_tl_x, preprocess_tl_y);
    srcPoints[1] = cv::Point2f(preprocess_tr_x, preprocess_tr_y);
    srcPoints[2] = cv::Point2f(preprocess_br_x, preprocess_br_y);
    srcPoints[3] = cv::Point2f(preprocess_bl_x, preprocess_bl_y);

    // Compute the transformation matrix
    cv::Mat transformationMatrix = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // Apply the perspective transformation
    warpPerspective(dst, dst, transformationMatrix, cv::Size(WIDTH, HEIGHT));
}
