#include "VisionPreprocessor.h"
#include "Mouse.h"

const int CLOSE = 20;

VisionPreprocessor::VisionPreprocessor()
{
    // Define the four corners of the bird's-eye view output image
    dstPoints[0] = cv::Point2f(0, 0);                             // Top-left corner
    dstPoints[1] = cv::Point2f(WIDTH, 0);                  // Top-right corner
    dstPoints[2] = cv::Point2f(WIDTH, HEIGHT - 100); // Bottom-right corner
    dstPoints[3] = cv::Point2f(0, HEIGHT - 100);  // Bottom-left corner

    first = true;
}

void VisionPreprocessor::Preprocess(cv::Mat& frame, cv::Mat& dst)
{
    // std::cout << "frame cols " << frame.cols << std::endl;
    // std::cout << "frame rows " << frame.rows << std::endl;

    if (first)
    {
        // Define the four corners of the region of interest (ROI) in the input image
        srcPoints[0] = cv::Point2f(frame.cols * 0.25, frame.rows * 0.03);                             // Top-left corner
        srcPoints[1] = cv::Point2f(frame.cols * 0.75, frame.rows * 0.03);                  // Top-right corner
        srcPoints[2] = cv::Point2f(frame.cols * 1.2, frame.rows); // Bottom-right corner
        srcPoints[3] = cv::Point2f(-frame.cols * 0.3, frame.rows);  // Bottom-left corner
    }

    // If the user left clicks and holds down shift near one of the corners
    if (Mouse::GetInstance().GetLeftDown())
    {
        cv::Point2f currMousePos = Mouse::GetInstance().GetPos();

        // Check each corner
        for (int i = 0; i < 4; i++)
        {
            if (down[i] || (cv::norm(dstPoints[i] - currMousePos) < CLOSE && shiftDown))
            {
                // Don't restrict adjustment once you start adjusting
                if (!down[i]) down[i] = true;

                srcPoints[i] -= currMousePos - _mousePosLast;
                // std::cout << i << std::endl;
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

    // for (int i = 0; i < 4; i++)
    // {
    //     std::cout << srcPoints[i].x << " " << srcPoints[i].y << std::endl;
    // }

    // Compute the transformation matrix
    cv::Mat transformationMatrix = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // Apply the perspective transformation
    warpPerspective(dst, dst, transformationMatrix, cv::Size(WIDTH, HEIGHT));


    SAFE_DRAW
    cv::circle(dst, _mousePosLast, 3, cv::Scalar(0, 255, 0), 2);
    END_SAFE_DRAW

    for (int i = 0; i < 4; i++)
    {
        SAFE_DRAW
        cv::circle(dst, dstPoints[i], 3, cv::Scalar(255, 0, 255), 2);
        END_SAFE_DRAW
    }


}
