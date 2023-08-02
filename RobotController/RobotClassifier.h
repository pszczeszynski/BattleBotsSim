/**
 * Used to descern between us and the opponent
 */

#pragma once

#include "RobotOdometry.h"

struct RobotCalibrationData
{
    cv::Scalar meanColor;
    cv::Mat histogram;
    double diameter;
};

// TODO: move this to it's own file
struct VisionClassification
{
    VisionClassification()
    {
        robotValid = false;
        opponentValid = false;
        hadNewImage = false;
    }

    void SetRobot(MotionBlob& robot)
    {
        robotValid = true;
        this->robot = robot;
    }

    void SetOpponent(MotionBlob& opponent)
    {
        opponentValid = true;
        this->opponent = opponent;
    }

    MotionBlob* GetRobotBlob()
    {
        if (!robotValid)
        {
            return nullptr;
        }
        return &robot;
    }

    MotionBlob* GetOpponentBlob()
    {
        if (!opponentValid)
        {
            return nullptr;
        }
        return &opponent;
    }

    void SetHadNewImage()
    {
        hadNewImage = true;
    }

    bool GetHadNewImage()
    {
        return hadNewImage;
    }

private:
    bool robotValid;
    bool opponentValid;
    bool hadNewImage;
    MotionBlob robot;
    MotionBlob opponent;
};

class RobotClassifier
{
public:
    static RobotClassifier* instance;
    RobotClassifier();

    VisionClassification ClassifyBlobs(std::vector<MotionBlob>& blobs, cv::Mat& frame, cv::Mat& motionImage);

    void SwitchRobots();
private:
    double ClassifyBlob(MotionBlob& blob, cv::Mat& frame, cv::Mat& motionImage);
    void RecalibrateRobot(RobotCalibrationData& data, MotionBlob& blob, cv::Mat& frame, cv::Mat& motionImage);
    cv::Scalar GetMeanColorOfBlob(MotionBlob& blob, cv::Mat& frame, cv::Mat& motionImage);

    RobotCalibrationData robotCalibrationData;
    RobotCalibrationData opponentCalibrationData;
};

