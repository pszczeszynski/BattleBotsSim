#include "CVRotation.h"
#include <opencv2/dnn.hpp>
#include "MathUtils.h"
#include "RobotController.h"
#include "UIWidgets/ClockWidget.h"
#include <opencv2/dnn/dnn.hpp>

CVRotation::CVRotation()
{
    // Load the model
    _net = cv::dnn::readNetFromONNX(MODEL_PATH);
    _net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    _net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
}

CVRotation& CVRotation::GetInstance()
{
    static CVRotation instance;
    return instance;
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

cv::Point2f ConvertNetworkOutputToXY(cv::Mat& output)
{
    // INTERPRET THE RESULTS
    float normalized_x = output.at<float>(0, 0);
    float normalized_y = output.at<float>(0, 1);

    // Convert normalized values back to the range [-1, 1]
    float x_component = (normalized_x * 2.0) - 1.0;
    float y_component = (normalized_y * 2.0) - 1.0;

    return cv::Point2f(x_component, y_component);
}

/**
 * \brief
 * Converts the output of the neural network to a rotation in radians
 * \param output The output of the neural network
 * \return The rotation in radians
*/
float ConvertNetworkOutputToRad(cv::Mat& output)
{
    cv::Point2f xy = ConvertNetworkOutputToXY(output);

    // Compute the angle using atan2
    float rotation = atan2(xy.y, xy.x);

    // Ensure the angle is within the desired range
    rotation = angle_wrap(rotation) * 0.5;

    return rotation;
}


#define CROP_SIZE 128

double CVRotation::ComputeRobotRotation(cv::Mat &fieldImage, cv::Point2f robotPos)
{
    static ClockWidget clock("CVRotation");

    clock.markStart();
    cv::Mat fieldImagePreprocessed;


    // convert to grayscale
    // Convert to gray scale 
    if (fieldImage.channels() == 1) {
        fieldImagePreprocessed = fieldImage.clone();
    }
    else if (fieldImage.channels() == 3) {
        cv::cvtColor(fieldImage, fieldImagePreprocessed, cv::COLOR_BGR2GRAY);
    }
    else if (fieldImage.channels() == 4)
    {
        cv::cvtColor(fieldImage, fieldImagePreprocessed, cv::COLOR_BGRA2GRAY);
    }

    // normalize the image
    fieldImagePreprocessed.convertTo(fieldImagePreprocessed, CV_32FC1, 1.0 / 255.0);

    // crop the image, but make sure we don't go out of bounds
    cv::Rect roi(robotPos.x - CROP_SIZE/2, robotPos.y - CROP_SIZE/2, CROP_SIZE, CROP_SIZE);
    cv::Mat croppedImage;
    _CropImage(fieldImagePreprocessed, croppedImage, roi);

    // flip the image horizontally for another prediction
    cv::Mat croppedImageFlipped;
    cv::flip(croppedImage, croppedImageFlipped, 1);

    // convert to blobs
    cv::Mat blob = cv::dnn::blobFromImage(croppedImage, 1.0, cv::Size(128, 128), cv::Scalar(0, 0, 0), false, false);
    cv::Mat blobFlip = cv::dnn::blobFromImage(croppedImageFlipped, 1.0, cv::Size(128, 128), cv::Scalar(0, 0, 0), false, false);

    // set the input to the network
    _net.setInput(blob);
    cv::Mat result1 = _net.forward();
    _netXY1 = ConvertNetworkOutputToXY(result1);
    float rotation1 = angle_wrap(ConvertNetworkOutputToRad(result1));

    // now, flip the image and do it again
    _net.setInput(blobFlip);
    cv::Mat result2 = _net.forward();
    _netXY2 = ConvertNetworkOutputToXY(result2);
    float rotation2 = angle_wrap(-ConvertNetworkOutputToRad(result2));

    // compute deltas for both the predicted rotation and the flipped rotation
    float diff = angle_wrap(rotation1 - rotation2);
    float diff2 = angle_wrap(rotation2 - angle_wrap(rotation1 - M_PI));

    // return whichever angle is closer to the last angle
    if (abs(diff2) < abs(diff))
    {
        rotation1 = angle_wrap(rotation1 - M_PI);
    }



    // compute the average of the two angles
    float avgAngle = InterpolateAngles(Angle(rotation1), Angle(rotation2), 0.5);


    // add 180 if closer to last rotation since the robot is symmetrical
    if (abs(angle_wrap(avgAngle - _lastRotation)) > M_PI / 2)
    {
        avgAngle = angle_wrap(avgAngle + M_PI);
    }

    // // if the angle changed by more than 15 degrees, blend it with the last angle
    // float deltaAngle = angle_wrap(avgAngle - _lastRotation);
    // const float THRESH_LARGE_CHANGE = 15 * TO_RAD;
    // if (abs(deltaAngle) > THRESH_LARGE_CHANGE)
    // {
    //     avgAngle = InterpolateAngles(Angle(_lastRotation), Angle(avgAngle), 0.3f);
    // }
    _lastDisagreementRad = abs(angle_wrap(rotation1 - rotation2));

    _lastRotation = avgAngle;

    clock.markEnd();

    return avgAngle;
}

/**
 * \brief
 * Gets the last computed rotation without recomputing it
*/
double CVRotation::GetLastComputedRotation()
{
    return _lastRotation;
}

double CVRotation::GetLastConfidence()
{
    // compute magnitude of _netXY1
    double mag1 = cv::norm(_netXY1);
    double mag2 = cv::norm(_netXY2);
    double magnitudeConfidence = (mag1 + mag2) / 2.0;

    // compute angle between _netXY1 and _netXY2
    const double MAX_DISAGREEMENT = 15 * TO_RAD;
    double angleConfidence = 1.0 - _lastDisagreementRad / MAX_DISAGREEMENT;
    angleConfidence = max(0.0, angleConfidence);

    return magnitudeConfidence * 1.0;// + angleConfidence * 0.3;
}