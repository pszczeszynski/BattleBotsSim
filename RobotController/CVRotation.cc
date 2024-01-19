#include "CVRotation.h"
#include <opencv2/dnn.hpp>
#include "MathUtils.h"

CVRotation::CVRotation()
{
    // Load the model
    _net = cv::dnn::readNetFromONNX(MODEL_PATH);
}

#define CROP_SIZE 128

double CVRotation::GetRobotRotation(cv::Mat &fieldImage, cv::Point2f robotPos)
{
    cv::Rect roi(robotPos.x - CROP_SIZE/2, robotPos.y - CROP_SIZE/2, CROP_SIZE, CROP_SIZE);

    // Ensure ROI is within image bounds
    if (roi.x < 0 || roi.y < 0 || roi.width != CROP_SIZE || roi.height != CROP_SIZE || 
        roi.x + roi.width > fieldImage.cols || roi.y + roi.height > fieldImage.rows || 
        robotPos.x < 0 || robotPos.y < 0 || robotPos.x > fieldImage.cols || robotPos.y > fieldImage.rows) 
    {
        return 0;
    }

    cv::Mat croppedImage = fieldImage(roi);
    // convert bgr to rgb
    cv::cvtColor(croppedImage, croppedImage, cv::COLOR_BGR2RGB);

    croppedImage.convertTo(croppedImage, CV_32FC3, 1.0 / 255.0);  // Normalize to [0, 1]


    cv::Mat blob = cv::dnn::blobFromImage(croppedImage, 1.0, cv::Size(128, 128), cv::Scalar(0, 0, 0), false, false);
    const int* sz = blob.size.p;

    _net.setInput(blob);
    cv::Mat result = _net.forward();

    float rotation = result.at<float>(0, 0) * 2 * M_PI;
    rotation = angle_wrap(rotation);


    // put arrow on train image
    cv::Point2f arrowEnd = cv::Point2f(64, 64) + cv::Point2f(100 * cos(rotation), 100 * sin(rotation));

    cv::arrowedLine(croppedImage, cv::Point2f(64, 64), arrowEnd, cv::Scalar(0, 0, 255), 2);

    return rotation;
}
