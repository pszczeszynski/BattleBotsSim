#pragma once
#include "RobotOdometry.h"

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
