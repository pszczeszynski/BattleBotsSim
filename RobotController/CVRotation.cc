#include "CVRotation.h"
#include <opencv2/dnn.hpp>
#include "MathUtils.h"
#include "RobotController.h"

CVRotation::CVRotation()
{
    // Load the model
    _net = cv::dnn::readNetFromONNX(MODEL_PATH);
    _posNet = cv::dnn::readNetFromONNX(POS_MODEL_PATH);
}

void CVRotation::_CropImage(cv::Mat &input, cv::Mat &cropped, cv::Rect roi)
{
    // Create an output image of the size of the ROI filled with black
    cropped = cv::Mat(roi.size(), input.type(), cv::Scalar::all(0));

    // Compute the overlapping region between the ROI and the input image
    cv::Rect validROI(
        max(roi.x, 0),
        max(roi.y, 0),
        min(roi.width - max(0, -roi.x), input.cols - max(roi.x, 0)),
        min(roi.height - max(0, -roi.y), input.rows - max(roi.y, 0)));

    // If there's an overlapping region, copy it to the corresponding location in the cropped image
    if (validROI.area() > 0)
    {
        input(validROI).copyTo(cropped(cv::Rect(validROI.x - roi.x, validROI.y - roi.y,
                                                validROI.width, validROI.height)));
    }
}

cv::Point2f GetPosPrediction(cv::Mat &fieldImage, cv::dnn::Net &posNet)
{
    cv::Mat resized = cv::Mat(360, 360, CV_8UC3, cv::Scalar(0, 0, 0));
    // resize to 360x360
    cv::resize(fieldImage, resized, cv::Size(360, 360));

    cv::imshow("fieldImage", resized);
    cv::waitKey(1);

    cv::Mat blob = cv::dnn::blobFromImage(resized, 1.0, cv::Size(360, 360), cv::Scalar(0, 0, 0), false, false);
    const int* sz = blob.size.p;

    posNet.setInput(blob);
    cv::Mat result = posNet.forward();

    float x = result.at<float>(0, 0) * 720;
    float y = (result.at<float>(0, 1)) * 720;

    return cv::Point2f(x, y);
}

float ConvertNetworkOutputToRad(cv::Mat& output)
{
    // INTERPRET THE RESULTS
    float normalized_cos = output.at<float>(0, 0);
    float normalized_sin = output.at<float>(0, 1);

    // Convert normalized values back to the range [-1, 1]
    float cos_component = (normalized_cos * 2.0) - 1.0;
    float sin_component = (normalized_sin * 2.0) - 1.0;

    // Compute the angle using atan2
    float rotation = atan2(sin_component, cos_component);

    // Ensure the angle is within the desired range
    rotation = angle_wrap(rotation) * 0.5;

    return rotation;
}


#define CROP_SIZE 128

double CVRotation::GetRobotRotation(cv::Mat &fieldImage, cv::Point2f robotPos)
{
    static double lastRotation = 0;

    cv::Mat fieldImagePreprocessed;
    // convert bgr to rgb
    cv::cvtColor(fieldImage, fieldImagePreprocessed, cv::COLOR_BGR2GRAY);

    // // equalize hist
    // cv::equalizeHist(fieldImagePreprocessed, fieldImagePreprocessed);
    // // blur just a little
    // cv::blur(fieldImagePreprocessed, fieldImagePreprocessed, cv::Size(2, 2));


    fieldImagePreprocessed.convertTo(fieldImagePreprocessed, CV_32FC1, 1.0 / 255.0);  // Normalize to [0, 1]

    // cv::Point2f robotPosPrediction = GetPosPrediction(fieldImagePreprocessed, _posNet);
    // cv::Mat& drawingImage = RobotController::GetInstance().GetDrawingImage();
    // // draw on image
    // cv::circle(drawingImage, robotPosPrediction, 5, cv::Scalar(0, 0, 255), -1);


    cv::Rect roi(robotPos.x - CROP_SIZE/2, robotPos.y - CROP_SIZE/2, CROP_SIZE, CROP_SIZE);

    cv::Mat croppedImage;
    _CropImage(fieldImagePreprocessed, croppedImage, roi);

    cv::Mat croppedImageFlipped;
    cv::flip(croppedImage, croppedImageFlipped, 1);

    cv::Mat blob = cv::dnn::blobFromImage(croppedImage, 1.0, cv::Size(128, 128), cv::Scalar(0, 0, 0), false, false);
    cv::Mat blobFlip = cv::dnn::blobFromImage(croppedImageFlipped, 1.0, cv::Size(128, 128), cv::Scalar(0, 0, 0), false, false);


    _net.setInput(blob);
    cv::Mat result1 = _net.forward();
    float rotation = ConvertNetworkOutputToRad(result1);

    _net.setInput(blobFlip);
    cv::Mat result2 = _net.forward();
    float rotation2 = angle_wrap(-ConvertNetworkOutputToRad(result2));

    float diff = angle_wrap(rotation - rotation2);
    float diff2 = angle_wrap(rotation2 - angle_wrap(rotation - M_PI));

    if (abs(diff2) < abs(diff))
    {
        rotation = angle_wrap(rotation - M_PI);
    }

    // if (abs(diff) > 15 * TO_RAD)
    // {
    //     return 0;
    // }


    float avgAngle = InterpolateAngles(Angle(rotation), Angle(rotation2), 0.5);

    // put arrow on train image
    cv::Point2f arrowEnd = cv::Point2f(64, 64) + cv::Point2f(100 * cos(rotation), 100 * sin(rotation));
    cv::arrowedLine(croppedImage, cv::Point2f(64, 64), arrowEnd, cv::Scalar(0, 0, 255), 2);

    // put arrow on train image
    arrowEnd = cv::Point2f(64, 64) + cv::Point2f(100 * cos(rotation2), 100 * sin(rotation2));
    cv::arrowedLine(croppedImage, cv::Point2f(64, 64), arrowEnd, cv::Scalar(0, 255, 0), 2);

    // show cropped image
    cv::imshow("cropped", croppedImage);
    cv::waitKey(1);


    // add 180 if closer to last rotation
    if (abs(angle_wrap(avgAngle - lastRotation)) > M_PI / 2)
    {
        avgAngle = angle_wrap(avgAngle + M_PI);
    }

    float deltaAngle = angle_wrap(avgAngle - lastRotation);
    const float THRESH_LARGE_CHANGE = 15 * TO_RAD;
    if (abs(deltaAngle) > THRESH_LARGE_CHANGE)
    {
        avgAngle = InterpolateAngles(Angle(lastRotation), Angle(avgAngle), 0.3f);
    }


    lastRotation = avgAngle;

    return avgAngle;
}
