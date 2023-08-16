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
#include "RobotOdometry.h"
#include <opencv2/core/core.hpp>

// #include "Graphics/GameLoop.h"
// #include <opencv2/cudaarithm.hpp>
// #include <opencv2/cudastereo.hpp>
#include "RobotClassifier.h"
#include "Globals.h"
#include "VisionPreprocessor.h"
#include "ValueBin.h"
#include "MatQueue.h"

class Vision
{
public:
    Vision(ICameraReceiver &overheadCam);
    VisionClassification ConsumeLatestClassification(cv::Mat& outDrawingImage);
    VisionPreprocessor& GetPreprocessor();

private:
    VisionClassification LocateRobots2d(cv::Mat&, cv::Mat&);
    VisionClassification RunPipeline(cv::Mat& currFrame);

    ICameraReceiver& overheadCam;
    cv::Mat previousBirdsEye;

    RobotClassifier robotClassifier;

    VisionPreprocessor birdsEyePreprocessor;
    std::thread processingThread;

    std::mutex _classificationMutex;
    cv::Mat _lastDrawingImage;
    VisionClassification _classification;
    Clock _prevFrameTimer;
};
