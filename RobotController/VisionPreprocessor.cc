#include "VisionPreprocessor.h"
#include "RobotConfig.h"
#include "UIWidgets/CameraWidget.h"

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

    // compute initial transformation matrix
    ComputeTransformationMatrix();
}

cv::Point2f VisionPreprocessor::TransformPoint(const cv::Point2f &pt)
{
    // Convert point to homogeneous coordinates
    cv::Mat srcMat = (cv::Mat_<double>(3, 1) << pt.x, pt.y, 1.0);

    // Apply transformation matrix
    cv::Mat dstMat = _transformationMatrix * srcMat;

    // Convert back to inhomogeneous coordinates
    float x = dstMat.at<double>(0, 0) / dstMat.at<double>(2, 0);
    float y = dstMat.at<double>(1, 0) / dstMat.at<double>(2, 0);

    return cv::Point2f(x, y);
}

void VisionPreprocessor::ComputeTransformationMatrix()
{
    cv::Point2f srcPoints[4];
    // Define the four corners of the bird's-eye view input image
    srcPoints[0] = cv::Point2f(preprocess_tl_x, preprocess_tl_y);
    srcPoints[1] = cv::Point2f(preprocess_tr_x, preprocess_tr_y);
    srcPoints[2] = cv::Point2f(preprocess_br_x, preprocess_br_y);
    srcPoints[3] = cv::Point2f(preprocess_bl_x, preprocess_bl_y);

    // Compute the transformation matrix
    _transformationMatrix = cv::getPerspectiveTransform(srcPoints, _dstPoints);
}

void VisionPreprocessor::Preprocess(cv::Mat &frame, cv::Mat &dst)
{
    // If frame is invalid, dont do it
    if( frame.empty() ) { return; }

    // Apply fish-eye removal
    cv::Mat map1, map2;
    cv::Mat outputImage;

    if( CameraWidget::DoFisheye)
    {
        _generateCameraParameters(FISHEYE_FL, FISHEYE_SCALE, FISHEYE_Y, frame.size(), K, D);

        // Compute the undistortion and rectification transformation map
        cv::Size newsize = frame.size();
        newsize.width = newsize.width * 1.25; // Correct the width for the expansion required
        cv::fisheye::initUndistortRectifyMap(K, D, cv::Matx33d::eye(), K, newsize, CV_16SC2, map1, map2);

        // Apply the undistortion and rectification transformation to the image
        cv::remap(frame, outputImage, map1, map2, cv::INTER_LINEAR);

        if( CameraWidget::ShowFisheyeImg)
        {
            cv::imshow("Fisheye Post", outputImage);
            cv::pollKey();
        }
    }
    else
    {
        outputImage = frame;
    }

    // recompute the transformation matrix
    ComputeTransformationMatrix();

    // Apply the perspective transformation
    warpPerspective(outputImage, dst, _transformationMatrix, cv::Size(WIDTH, HEIGHT), cv::INTER_NEAREST);

#ifdef STABALIZE
    _StabalizeImage(dst, dst);
#endif

    
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

// Function to generate a guess at the camera parameters
void VisionPreprocessor::_generateCameraParameters(float scaler_fl, float scaler_intensity, float scaler_y, cv::Size imageSize, cv::Matx33d &K, cv::Vec4d &D) 
{
    // Assume the focal lengths (fx, fy) are proportional to the scaler
    double fx = 2.8 * scaler_fl;  // Focal length in x direction
    double fy = 2.8 * scaler_fl;  // Focal length in y direction

    // Assume the principal point (cx, cy) is at the center of the image
    double cx = imageSize.width/2.0;  // Principal point in x direction
    double cy = imageSize.height*scaler_y;  // Principal point in y direction

    // Set the camera matrix (Intrinsic matrix)
    K = cv::Matx33d(fx,  0, cx,
                     0, fy, cy,
                     0,  0,  1);

    // Assume the distortion coefficients (k1, k2, p1, p2) are proportional to the scaler
    double k1 = 0.1 * scaler_intensity;  // Radial distortion coefficient
    double k2 = 0.01 * scaler_intensity; // Radial distortion coefficient
    double p1 = 0.001 * scaler_intensity; // Tangential distortion coefficient
    double p2 = 0.001 * scaler_intensity; // Tangential distortion coefficient

    // Set the distortion coefficients
    D = cv::Vec4d(k1, k2, p1, p2);
}