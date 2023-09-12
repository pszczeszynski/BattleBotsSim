#include "VisionPreprocessor.h"
#include "RobotConfig.h"
#include "Clock.h"

const int CLOSE = 20;

// #define STABALIZE

VisionPreprocessor::VisionPreprocessor()
{
    // Define the four corners of the bird's-eye view output image
    _dstPoints[0] = cv::Point2f(0, 0);          // Top-left corner
    _dstPoints[1] = cv::Point2f(WIDTH, 0);      // Top-right corner
    _dstPoints[2] = cv::Point2f(WIDTH, HEIGHT); // Bottom-right corner
    _dstPoints[3] = cv::Point2f(0, HEIGHT);     // Bottom-left corner

    _prevFrame = cv::Mat(WIDTH, HEIGHT, CV_8UC3);
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
    cv::Mat transformationMatrix = cv::getPerspectiveTransform(srcPoints, _dstPoints);

    Clock c;
    // Apply the perspective transformation
    warpPerspective(dst, dst, transformationMatrix, cv::Size(WIDTH, HEIGHT), cv::INTER_NEAREST);

    // // split channels
    // std::vector<cv::Mat> channels;
    // cv::split(dst, channels);

    // // equalize histogram of each
    // cv::equalizeHist(channels[0], channels[0]);
    // cv::equalizeHist(channels[1], channels[1]);
    // cv::equalizeHist(channels[2], channels[2]);

    // // merge channels
    // cv::merge(channels, dst);

    // Clock c;
    // stabilize the image
#ifdef STABALIZE
    _StabalizeImage(dst, dst);
#endif

    // std::cout << "time in ms: " << c.getElapsedTime() * 1000 << std::endl;
}

void VisionPreprocessor::_StabalizeImage(cv::Mat &frame, cv::Mat &dst)
{
    cv::Rect roi = cv::Rect(WIDTH * 0.1, HEIGHT * 0.1, WIDTH / 4, HEIGHT / 4);

    // if the prev frame is empty
    if (_prevFrame.empty())
    {
        // copy frame to prev frame
        frame.copyTo(_prevFrame);
        return;
    }

    // Clock c;
    // compute the translation
    cv::Point2f translation = _TrackFeature(_prevFrame, frame, roi);

    // std::cout << "track feature time: " << c.getElapsedTime() * 1000 << std::endl;

    // c.markStart();
    // translate and clip the entire image
    cv::Mat translationMatrix = (cv::Mat_<double>(2, 3) << 1, 0, translation.x, 0, 1, translation.y);
    cv::warpAffine(frame, dst, translationMatrix, cv::Size(WIDTH, HEIGHT));

    // std::cout << "warp affine time: " << c.getElapsedTime() * 1000 << std::endl;
    // c.markStart();

    // copy dst to the prev frame
    dst.copyTo(_prevFrame);
    
    // std::cout << "copy time: " << c.getElapsedTime() * 1000 << std::endl;

    // draw the roi on dst
    cv::rectangle(dst, roi, cv::Scalar(0, 0, 255), 2);
}

#define SEARCH_WINDOW 3
cv::Point2f VisionPreprocessor::_TrackFeature(cv::Mat& prevFrame, cv::Mat& frame, cv::Rect& roi)
{
    // crop the frame
    cv::Mat frameCropped = frame(roi);

    cv::Rect currRoi = roi;
    cv::Point2f currTranslation = cv::Point2f(0, 0);
    double minSum = std::numeric_limits<double>::max();
    cv::Point2f bestTranslation = cv::Point2f(0, 0);

    // search around for the best translation
    for (currTranslation.y = -SEARCH_WINDOW; currTranslation.y <= SEARCH_WINDOW; currTranslation.y++)
    {
        for (currTranslation.x = -SEARCH_WINDOW; currTranslation.x <= SEARCH_WINDOW; currTranslation.x++)
        {
            // translate roi by translation into currRoi
            currRoi.x = roi.x + currTranslation.x;
            currRoi.y = roi.y + currTranslation.y;

            // crop the previous frame
            cv::Mat prevFrameCropped = prevFrame(currRoi);

            // compute the difference
            cv::Mat diff;
            cv::absdiff(prevFrameCropped, frameCropped, diff);

            // compute the sum of the difference
            double sum = cv::sum(diff)[0];

            // if the sum is less than the current minimum
            if (sum < minSum)
            {
                // update the minimum
                minSum = sum;

                // update the best translation
                bestTranslation = currTranslation;
            }
        }
    }

    return bestTranslation;
}