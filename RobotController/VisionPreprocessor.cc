#include "VisionPreprocessor.h"

const int CLOSE = 20;

VisionPreprocessor::VisionPreprocessor()
{
    // Define the four corners of the bird's-eye view output image
    dstPoints[0] = cv::Point2f(0, 0);          // Top-left corner
    dstPoints[1] = cv::Point2f(WIDTH, 0);      // Top-right corner
    dstPoints[2] = cv::Point2f(WIDTH, HEIGHT); // Bottom-right corner
    dstPoints[3] = cv::Point2f(0, HEIGHT);     // Bottom-left corner

    srcPoints[0] = cv::Point2f(0, 0);                  // Top-left corner
    srcPoints[1] = cv::Point2f(WIDTH, 0);              // Top-right corner
    srcPoints[2] = cv::Point2f(WIDTH, HEIGHT * 2 / 3); // Bottom-right corner
    srcPoints[3] = cv::Point2f(0, HEIGHT * 2 / 3);     // Bottom-left corner
}

/**
 * @brief Allows moving the source points of the perspective transform
*/
void VisionPreprocessor::MoveSourcePoint(int index, cv::Point2f displacement)
{
    srcPoints[index] += displacement;
}

void VisionPreprocessor::Preprocess(cv::Mat &frame, cv::Mat &dst)
{
    // Compute the transformation matrix
    cv::Mat transformationMatrix = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // Apply the perspective transformation
    warpPerspective(dst, dst, transformationMatrix, cv::Size(WIDTH, HEIGHT));

    SAFE_DRAW
    dst.copyTo(P_DRAWING_IMAGE);
    END_SAFE_DRAW
}
