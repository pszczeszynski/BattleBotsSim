#include "VisionPreprocessor.h"
#include "Mouse.h"

const int CLOSE = 20;

VisionPreprocessor::VisionPreprocessor()
{
    // Define the four corners of the bird's-eye view output image
    dstPoints[0] = cv::Point2f(0, 0);                             // Top-left corner
    dstPoints[1] = cv::Point2f(WIDTH, 0);                  // Top-right corner
    dstPoints[2] = cv::Point2f(WIDTH, HEIGHT); // Bottom-right corner
    dstPoints[3] = cv::Point2f(0, HEIGHT);  // Bottom-left corner

    first = true;
}


void VisionPreprocessor::Preprocess(cv::Mat& frame, cv::Mat& dst)
{
    if (first)
    {
        // Define the four corners of the region of interest (ROI) in the input image
        srcPoints[0] = cv::Point2f(frame.cols * 0.25, frame.rows * 0.03); // Top-left corner
        srcPoints[1] = cv::Point2f(frame.cols * 0.75, frame.rows * 0.03); // Top-right corner
        srcPoints[2] = cv::Point2f(frame.cols * 1.2, frame.rows);         // Bottom-right corner
        srcPoints[3] = cv::Point2f(-frame.cols * 0.3, frame.rows);        // Bottom-left corner
    }
    cv::Point2f currMousePos = Mouse::GetInstance().GetPos();

    // If the user left clicks and holds down shift near one of the corners
    if (nearCorner && Mouse::GetInstance().GetLeftDown())
    {
        // Check each corner
        for (int i = 0; i < 4; i++)
        {
            if (down[i] || (cv::norm(dstPoints[i] - currMousePos) < CLOSE))
            {
                // Don't restrict adjustment once you start adjusting
                if (!down[i]) down[i] = true;

                // srcPoints[i] -= currMousePos - _mousePosLast;
                first = false;
                break;
            }
        }
    }
    else
    {
        for (int i = 0; i < 4; i++) down[i] = false;
    }

    _mousePosLast = Mouse::GetInstance().GetPos();


    // Compute the transformation matrix
    cv::Mat transformationMatrix = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // Apply the perspective transformation
    warpPerspective(dst, dst, transformationMatrix, cv::Size(WIDTH, HEIGHT));

    SAFE_DRAW
    dst.copyTo(P_DRAWING_IMAGE);
    END_SAFE_DRAW

    SAFE_DRAW
    cv::circle(drawingImage, _mousePosLast, 3, cv::Scalar(0, 255, 0), 2);
    END_SAFE_DRAW
    
    bool outside = true;
    for (int i = 0; i < 4; i++)
    {
        SAFE_DRAW
        if (cv::norm(dstPoints[i] - currMousePos) < CLOSE)
        {
            cv::circle(drawingImage, dstPoints[i], CLOSE * 1.5, cv::Scalar(255, 100, 255), 2);
            nearCorner = true;
            outside = false;
        }
        else
        {
            cv::circle(drawingImage, dstPoints[i], CLOSE, cv::Scalar(255, 0, 255), 2);
            if (down[i]) outside = false;
        }
        END_SAFE_DRAW
    }

    if (outside) nearCorner = false;
}
