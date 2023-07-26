#pragma once

#include <opencv2/core/core.hpp>

class Mouse
{
// singleton
public:
    static Mouse& GetInstance();
    void SetLeftDown(bool down);
    void SetRightDown(bool down);
    bool GetLeftDown();
    bool GetRightDown();
    void SetPos(cv::Point2f pos);
    cv::Point2f GetPos();

    void DrawCursor();
private:
    Mouse() {};
    Mouse(Mouse const&) = delete;
    void operator=(Mouse const&) = delete;

    bool leftDown;
    bool rightDown;
    cv::Point2f pos;
};