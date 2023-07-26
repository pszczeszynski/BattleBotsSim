#include "Mouse.h"
#include <iostream>
#include "Globals.h"

std::mutex lock;
Mouse& Mouse::GetInstance()
{
    static Mouse instance;
    return instance;
}

void Mouse::SetLeftDown(bool down)
{
    lock.lock();
    leftDown = down;
    lock.unlock();
}

void Mouse::SetRightDown(bool down)
{
    lock.lock();
    rightDown = down;
    lock.unlock();
}

bool Mouse::GetLeftDown()
{
    lock.lock();
    bool ret = leftDown;
    lock.unlock();
    return ret;
}

bool Mouse::GetRightDown()
{
    lock.lock();
    bool ret = rightDown;
    lock.unlock();
    return ret;}

void Mouse::SetPos(cv::Point2f pos)
{
    lock.lock();
    this->pos = pos;
    lock.unlock();
}

cv::Point2f Mouse::GetPos()
{
    lock.lock();
    cv::Point2f ret = pos;
    lock.unlock();
    return ret;
}

void Mouse::DrawCursor()
{
    lock.lock();
    cv::circle(DRAWING_IMAGE, pos, 5, cv::Scalar(255, 255, 255), 2);
    lock.unlock();
}