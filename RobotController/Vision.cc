#include "Vision.h"
#include "MathUtils.h"
#include "RobotStateParser.h"

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include "Graphics/GameLoop.h"

// GLint doesn't have different sizes on different compilers whereas int does
const GLint POINT_CLOUD_WINDOW_WIDTH = 1920, POINT_CLOUD_WINDOW_HEIGHT = 1080;


#define WIDTH 640
#define HEIGHT 480

#define HEIGHT_DISPARITY WIDTH * 0.6

#define DISPARITY_SCALAR 3000.0

// camera properties obtained from unity
#define FOV_X TO_RAD * 75.18 * 2
#define FOV_Y TO_RAD * 60.0 * 2

const int NUM_DISPARITIES = 200;
Vision::Vision() : sgbm(cv::StereoSGBM::create())
{
    // 1. setup StereoSGBM
    int channels = 3;
    // Define the stereo matching method and parameters
    sgbm->setMinDisparity(0);
    sgbm->setDisp12MaxDiff(0);
    sgbm->setNumDisparities(NUM_DISPARITIES);
    sgbm->setBlockSize(5);
    sgbm->setP1(2 * channels * 3 * 3);
    sgbm->setP2(4 * channels * 3 * 3); // increasing makes sortof smoother -> more blobby

    pointCloudThread = new std::thread([this]()
    {
        const Engine::WindowSettings myWindowSettings = {POINT_CLOUD_WINDOW_WIDTH, POINT_CLOUD_WINDOW_HEIGHT};
        Engine::Window myWindow = Engine::Window(myWindowSettings);
        myWindow.Show();

        // this will handle all the logic of our game
        GameLoop gameLoop(WIDTH, HEIGHT, &myWindow);
        pGameLoop = &gameLoop;

        // bind gameLoop's update function to the window
        myWindow.addLoopFunction([&gameLoop]()
                                { gameLoop.update(); });
        myWindow.addInitFunction([&gameLoop]()
                                { gameLoop.init(); });
        // this will start calling the gameLoop update and also polling events
        myWindow.startLoop();
    });
}

void extendImageLeftSide(const cv::Mat &src, cv::Mat &dst, int width)
{
    // Create an image with the added black rectangle
    dst = cv::Mat(src.rows, src.cols + width, CV_8UC3, cv::Scalar(0, 0, 0));
    src.copyTo(dst(cv::Rect(width, 0, src.cols, src.rows)));
}

void Vision::computeDisparity(const cv::Mat &left, const cv::Mat &right, cv::Mat &disparity)
{
    cv::Mat leftCrop = left(cv::Rect(cv::Point2f(0, 0), cv::Point2f(WIDTH, HEIGHT_DISPARITY)));
    cv::Mat rightCrop = right(cv::Rect(cv::Point2f(0, 0), cv::Point2f(WIDTH, HEIGHT_DISPARITY)));

    // extend the left side of the images with black. This is because the greater
    // NUM_DISPARITIES, the greater crop on the left side the disparity map will have.
    cv::Mat leftCropExtended;
    extendImageLeftSide(leftCrop, leftCropExtended, NUM_DISPARITIES);
    cv::Mat rightCropExtended;
    extendImageLeftSide(rightCrop, rightCropExtended, NUM_DISPARITIES);

    // Compute the disparity map
    sgbm->compute(leftCropExtended, rightCropExtended, disparity);

    // Crop the left side of the image
    disparity = disparity(cv::Rect(NUM_DISPARITIES, 0, disparity.cols - NUM_DISPARITIES, disparity.rows));
    // medianBlur(disparity, disparity, 5);

    cv::Mat disparityNormalized;
    // Normalize the disparity map
    cv::normalize(disparity, disparityNormalized, 0, 255, cv::NORM_MINMAX, CV_8U);

    cv::imshow("Disparity Normalized", disparityNormalized);
    cv::waitKey(1);
}

Point Vision::convert2dPointTo3d(int x, int y, short disparity)
{
    // calc normalized positions
    // 1 means fully to the top of the image or to the right
    // -1 means fully to the bottom of the image or to the left
    double xNormalized = (x - (WIDTH / 2.0)) / (WIDTH / 2.0);
    double yNormalized = -(y - (HEIGHT / 2.0)) / (HEIGHT / 2.0);
    double zPos = DISPARITY_SCALAR / (disparity);

    Point point{xNormalized, yNormalized, zPos * 1.3};

    point.x = xNormalized * sin(FOV_X / 2) * zPos;
    point.y = yNormalized * sin(FOV_Y / 2) * zPos;

    point.x *= 500.0;
    point.y *= 500.0;
    point.z *= 500.0;

    return point;
}

// dense version
void Vision::compute3dPointCloud(const cv::Mat &leftCam, const cv::Mat &rightCam, std::vector<Point> &pointCloud)
{
    cv::Mat disparity;
    computeDisparity(leftCam, rightCam, disparity);

    // corresponding colors of all the points
    std::vector<cv::Vec3b> colors = {};

    for (int y = 0; y < disparity.rows; y += 1)
    {
        for (int x = 0; x < disparity.cols; x += 1)
        {
            short pixel = disparity.at<short>(y, x);

            // only send pixels with disparities greater than 0
            if (pixel > 0)
            {
                colors.push_back(leftCam.at<cv::Vec3b>(y, x));
                Point p = convert2dPointTo3d(x, y, pixel);
                pointCloud.push_back(p);
            }
        }
    }

    if (pGameLoop)
    {
        // TODO: can't just directly pass them, must copy them into buffer, then other thread reads them.
        pGameLoop->SetPointCloudVerts(pointCloud, colors);
    }
}
