#include "VisionPreprocessor.h"
#include "RobotConfig.h"

#ifndef MACHINE_LEARNING
#include "UIWidgets/CameraWidget.h"
#endif
#include "MathUtils.h"

#ifndef MACHINE_LEARNING
#include "UIWidgets/ClockWidget.h"
#endif

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

    // Initialize our dstPointsList. Also populate srcPointList so we dont have to do it later
    for( float y=0; y <= gridSize-1; y += 1.0)
    {
        for( float x=0; x <= gridSize-1; x += 1.0)
        {
            std::vector<cv::Point2f> innerRec;
            innerRec.push_back(cv::Point2f(x*WIDTH/gridSize, y*HEIGHT/gridSize));
            innerRec.push_back(cv::Point2f((x+1)*WIDTH/gridSize, y*HEIGHT/gridSize));
            innerRec.push_back(cv::Point2f((x+1)*WIDTH/gridSize, (y+1)*HEIGHT/gridSize));
            innerRec.push_back(cv::Point2f(x*WIDTH/gridSize, (y+1)*HEIGHT/gridSize));
            dstPointsList.push_back(innerRec);

            std::vector<cv::Point2f> innerRecCopy(innerRec);
            srcPointsList.push_back(innerRecCopy);
        }
    }


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
#ifndef MACHINE_LEARNING
    preprocessClock.markStart();
#endif

    // If frame is invalid, dont do it
    if( frame.empty() ) { return; }

    // Apply fish-eye removal
    cv::Mat map1, map2;
    cv::Mat outputImage;

/*  // Old fisheye code that is too slow
    if( false &&  FISHEYE_ENABLE)
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
*/
    outputImage = frame;

#ifdef STABALIZE
    _StabalizeImage(dst, dst);
#endif

    // recompute the transformation matrix
    ComputeTransformationMatrix();

    // Apply the perspective transformation
    warpPerspective(outputImage, dst, _transformationMatrix, cv::Size(WIDTH, HEIGHT), cv::INTER_NEAREST);

#ifndef MACHINE_LEARNING
    // Apply poor-mans fisheye correction
    if( FISHEYE_ENABLE)
    {
        // Get the src points
        ComputePMFisheyePointList(dst);

        // Do the Fisheye correctoin
        cv::Mat correctedImage;
        DoPMFishEye(dst, correctedImage);

        if( CameraWidget::tuningMode)
        {
            _FIImageShown = true;
            cv::imshow("Fisheye Post", correctedImage);
            cv::pollKey();
        }
        else if(_FIImageShown )
        {
            cv::destroyWindow("Fisheye Post");
            _FIImageShown = false;
        }

        if( ! CameraWidget::tuningMode)
        {   
            correctedImage.copyTo(dst);
        }
    }


    preprocessClock.markEnd();
#endif

}

#ifndef MACHINE_LEARNING
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

void VisionPreprocessor::ComputePMFisheyePointList(cv::Mat& image)
{
    // Create the src pointlist
    for(size_t i = 0; i < dstPointsList.size(); i++)
    {
        for(size_t j =0; j < dstPointsList[i].size(); j++)
        {
            srcPointsList[i][j] = GetPMFisheyeStartPoint(dstPointsList[i][j]);
        }
    }

    // Now display it
    if( CameraWidget::tuningMode)
    {
        for(size_t i = 0; i < srcPointsList.size(); i++)
        {
            cv::line(image, srcPointsList[i][0], srcPointsList[i][1], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            cv::line(image, srcPointsList[i][1], srcPointsList[i][2], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            cv::line(image, srcPointsList[i][2], srcPointsList[i][3], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            cv::line(image, srcPointsList[i][3], srcPointsList[i][0], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        }
    }
}

cv::Point2f VisionPreprocessor::GetPMFisheyeStartPoint(cv::Point2f& point)
{
    // FISHEYE_FL, FISHEYE_SCALE, FISHEYE_Y
    cv::Point2f center( WIDTH/2.0, HEIGHT*FISHEYE_Y);

    // Calculate the length
    double regRadius = norm(point - center);

    if(regRadius < 0.01f)
    {
        return center;
    }
    // Get the radius along the distorted image
    // double disRadius = 1000/FISHEYE_FL * atan(2*regRadius*tan(FISHEYE_FL/2000));
    double disRadius = 4.0 * FISHEYE_FL * sin(atan(regRadius/FISHEYE_FL/2.0)/2.0);
    return (center + FISHEYE_SCALE/10.0f * disRadius/regRadius * (point - center));

}


// Poor-Mans Fish-eye 
// Ok so this Poor-Mans fish-eye removal takes > 20ms single-threaded, but it can be threaded as opposed to a single remap call
void VisionPreprocessor::DoPMFishEye(cv::Mat& inImage, cv::Mat& outImage)
{
    // Since inImage may = outImage, will first create a buffer
    cv::Mat dst = cv::Mat::zeros(inImage.size(), inImage.type());

    int processes_run = 0;  
    int processes_done = 0;

    for (size_t i = 0; i < srcPointsList.size(); ++i) 
    {
        
        ++processes_run;
        auto boundFunction = std::bind(&VisionPreprocessor::PMFishEyeCoreThread, this, i, std::ref(inImage), std::ref(dst), 
                                        std::ref(processes_done), std::ref(_cvFisheye), std::ref(_mutexFisheye));

        ThreadPool::myThreads.enqueue(boundFunction);
    }
    
    _mutexFisheye.lock();
    while (processes_done < processes_run)
    {
        if (_cvFisheye.wait_for(_mutexFisheye, std::chrono::milliseconds(200)) == std::cv_status::timeout)
        {
            std::cout << "Timeout occured in Fisheye Processing " << std::endl;
        }
    }
    _mutexFisheye.unlock();

    // Output final image
    dst.copyTo(outImage);
}

void VisionPreprocessor::PMFishEyeCoreThread(size_t i, cv::Mat& inImage, cv::Mat& dst, int &doneInt, std::condition_variable_any &doneCV, std::mutex &mutex)
{
        MultiThreadCleanup cleanup(doneInt, mutex, doneCV);

        // Compute the perspective transform matrix
        cv::Mat M = cv::getPerspectiveTransform(srcPointsList[i], dstPointsList[i]);

        // Apply the perspective transformation to the source image
        cv::Mat warped;
        cv::warpPerspective(inImage, warped, M, dst.size());

        // Create a mask from the warped image
        cv::Mat mask = cv::Mat::zeros(warped.size(), CV_8UC1);
        cv::fillConvexPoly(mask, convertPoints(dstPointsList[i]), cv::Scalar(255));

        // Copy the warped image into the destination image using the mask to restrict the copy operation to the transformed trapezoid
        _mutexDrawImage.lock();
        warped.copyTo(dst, mask);
        _mutexDrawImage.unlock();
}

#endif
std::vector<cv::Point> VisionPreprocessor::convertPoints(const std::vector<cv::Point2f>& points2f)
{
    std::vector<cv::Point> points;
    for (const auto& point2f : points2f)
    {
        points.push_back(cv::Point(static_cast<int>(roundf(point2f.x)), static_cast<int>(roundf(point2f.y))));
    }
    return points;
}