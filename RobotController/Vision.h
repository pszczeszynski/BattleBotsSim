#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "MathUtils.h"
#include "RobotStateParser.h"
#include "CameraReceiver.h"
#include "RobotTracker.h"
#include "OpticalFlow.h"
#include "PathFinder.h"
#include "Graphics/GameLoop.h"
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudastereo.hpp>
#include "RobotClassifier.h"
#include "Globals.h"

class Vision
{
public:
    Vision(ICameraReceiver &overheadCam);
    void locateRobots2d(cv::Mat&, cv::Mat&);
    void runPipeline();
    void convertToBirdsEyeView2d(cv::Mat& frame, cv::Mat& dst);
    const cv::Mat& GetBirdsEyeImage();

    cv::Point2f GetRobotPosition();
    double GetRobotAngle();
    cv::Point2f GetOpponentPosition();
    double GetOpponentAngle();

    void DetectRotation(cv::Mat& canny);
    bool areMatsEqual(const cv::Mat &mat1, const cv::Mat &mat2);

    double angle;
    cv::Point2f position;
    double opponent_angle;
    cv::Point2f opponent_position;
private:
    ICameraReceiver& overheadCam;
    cv::Mat currFrame;
    cv::Mat previousBirdsEye;

    RobotTracker robotTracker;
    RobotTracker opponentTracker;

    RobotClassifier robotClassifier;
};
