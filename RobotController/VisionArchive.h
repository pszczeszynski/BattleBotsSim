#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "MathUtils.h"
#include "CameraReceiver.h"
#include "RobotOdometry.h"
#include "OpticalFlow.h"
#include "PathFinder.h"
#include "Graphics/GameLoop.h"
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudastereo.hpp>

// Function to build a kd-tree from an array of cv::Point3f points
cv::flann::Index buildKDTree(const std::vector<cv::Point3f> &points)
{
    cv::Mat data(points.size(), 3, CV_32F);
    for (size_t i = 0; i < points.size(); ++i)
    {
        data.at<float>(i, 0) = points[i].x;
        data.at<float>(i, 1) = points[i].y;
        data.at<float>(i, 2) = points[i].z;
    }

    cv::flann::Index index(data, cv::flann::KDTreeIndexParams());
    index.build(data, cv::flann::KDTreeIndexParams());
    return index;
}

void DetectBottomLineAndShowImage(cv::Mat& linesImage)
{
    // convert lines to 3 channels
    cv::cvtColor(linesImage, linesImage, cv::COLOR_GRAY2BGR);
    cv::Point2f currPos = mousePos;
    int step_size = 3;

    for (int step = 0; step < 50; step++)
    {
        bool found = false;
        for (double nextYDelta = 10; nextYDelta > -20; nextYDelta -= 1)
        {
            cv::Point2f projected = currPos + cv::Point2f(step_size, nextYDelta);
            // break if off the image
            if (projected.x < 0 || projected.x >= linesImage.cols || projected.y < 0 || projected.y >= linesImage.rows)
            {
                std::cout << "out of bounds" << std::endl;
                break;
            }

            if (linesImage.at<cv::Vec3b>(projected)[0] > 0)
            {
                cv::line(linesImage, currPos, projected, cv::Scalar(0, 0, 255), 3);
                currPos = projected;
                std::cout << "added: " << projected << std::endl;
                found = true;
                break;
            }
        }
        if (!found)
        {
            std::cout << "not found" << std::endl;
            break;
        }
    }

    // get location of mouse
    cv::namedWindow("Image");
    cv::setMouseCallback("Image", onMouse);
    cv::imshow("Image", linesImage);
}

void detectLines(cv::Mat& mat, cv::Mat& dst)
{
    // cv::Canny(mat, dst, 50, 200, 3);
    cv::Canny(mat, dst, 30, 90, 3);

    // dilate
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(dst, dst, element);

    // cv::Canny(dst, dst, 30, 90, 3);

    // cv::imshow("canny", dst);

    return;

    int LOOK_SIZE = 20;
    for (int x = 0; x < dst.cols; x++)
    {
        for (int y = 0; y < dst.rows - LOOK_SIZE; y++)
        {
            // check if we have a pixel
            cv::Vec3b color = dst.at<cv::Vec3b>(y, x);
            if (color[0] + color[1] + color[2] == 0)
            {
                continue;
            }

            // take the max of the pixels below us (LOOK_SIZE)
            int belowCount = 0;
            for (int i = 1; i < LOOK_SIZE; i++)
            {
                cv::Vec3b color = dst.at<cv::Vec3b>(y + i, x);
                int sum = color[0] + color[1] + color[2];
                if (sum > 0)
                {
                    belowCount ++;
                }
            }

            if (belowCount > 2)
            {
                // set the pixel to black
                dst.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
            }
        }
    }
}


cv::Point2f mousePos = cv::Point2f(0, 0);
void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        // Left button of the mouse is clicked
        std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
        mousePos = cv::Point2f(x, y);
    }
}
