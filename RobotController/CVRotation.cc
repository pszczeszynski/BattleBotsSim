#include "CVRotation.h"
#include <opencv2/dnn.hpp>
#include "MathUtils.h"
#include "RobotController.h"
#include "UIWidgets/ClockWidget.h"
#include <opencv2/dnn/dnn.hpp>
#include "RobotConfig.h"
CVRotation* CVRotation::_instance = nullptr;

CVRotation::CVRotation(ICameraReceiver* videoSource) : OdometryBase(videoSource)
{
    // Load the model
    _net = cv::dnn::readNetFromONNX(MODEL_PATH);
    _net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    _net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    _instance = this;
}

CVRotation* CVRotation::GetInstance()
{
    return _instance;
}

void CVRotation::_ProcessNewFrame(cv::Mat frame, double frameTime)
{
    cv::Point2f robotPos = RobotController::GetInstance().odometry.Robot().robotPosition;
    double rotation = ComputeRobotRotation(frame, robotPos);
    double conf = GetLastConfidence();

    if (conf < ANGLE_FUSE_CONF_THRESH)
    {
        return;
    }

    _frameID++;

    std::unique_lock<std::mutex> lock(_updateMutex);
    _currDataRobot.Clear();
    _currDataRobot.isUs = true;
    _currDataRobot.robotAngleValid = true;
    _currDataRobot.robotAngle = Angle(rotation);
    _currDataRobot.id = _frameID;
    _currDataRobot.time = frameTime;
    _currDataRobot.time_angle = frameTime;
    lock.unlock();
}

bool CVRotation::_CropImage(cv::Mat &input, cv::Mat &cropped, cv::Rect roi)
{
    // Create an output image of the size of the ROI filled with black
    cropped = cv::Mat(roi.size(), input.type(), cv::Scalar::all(0));

    // Define the image boundaries
    cv::Rect image_rect(0, 0, input.cols, input.rows);

    // Compute the overlapping region between the ROI and the input image
    cv::Rect validROI = roi & image_rect;

    // Check if there's any overlapping region
    if (validROI.area() > 0)
    {
        // Compute the destination ROI in the cropped image
        cv::Rect destROI(validROI.x - roi.x, validROI.y - roi.y, validROI.width, validROI.height);

        // Copy the overlapping region from the input image to the cropped image
        input(validROI).copyTo(cropped(destROI));

        return true; // Valid image data exists within the crop
    }
    else
    {
        // No valid image data within the crop
        return false;
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
    bool success = _CropImage(fieldImagePreprocessed, croppedImage, roi);

    if (!success)
    {
        clock.markEnd();
        // if the crop failed, return the last rotation
        return _lastRotation;
    }

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



    
    // the guiding Angle is either our last rotation or the imu. This is used to decide the +- 180 degree ambiguity
    double guidingAngle = RobotController::GetInstance().odometry.Robot().robotAngle;
    // add 180 if closer to last rotation since the robot is symmetrical
    if (abs(angle_wrap(avgAngle - guidingAngle)) > M_PI / 2)
    {
        avgAngle = angle_wrap(avgAngle + M_PI);
    }

    // if the angle changed by more than 15 degrees, blend it with the last angle
    float deltaAngle = angle_wrap(avgAngle - _lastRotation);
    const float THRESH_LARGE_CHANGE = 15 * TO_RAD;
    if (abs(deltaAngle) > THRESH_LARGE_CHANGE)
    {
        avgAngle = InterpolateAngles(Angle(_lastRotation), Angle(avgAngle), 0.5f);
    }
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