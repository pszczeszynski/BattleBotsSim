#include "Vision.h"
#include "MathUtils.h"
#include "RobotStateParser.h"

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>

// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>


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

    std::cout << "done" << std::endl;
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

    cv::Mat disparityNormalized;
    // Normalize the disparity map
    cv::normalize(disparity, disparityNormalized, 0, 255, cv::NORM_MINMAX, CV_8U);

    // medianBlur(disparity, disparity, 5);
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

    Point point{xNormalized, yNormalized, zPos};

    point.x = xNormalized * sin(FOV_X / 2) * zPos;
    point.y = yNormalized * sin(FOV_Y / 2) * zPos;

    point.x *= 500.0;
    point.y *= 500.0;
    point.z *= 500.0;

    // point.y += 100.0;
    return point;
}

// dense version
void Vision::compute3dPointCloud(const cv::Mat &leftCam, const cv::Mat &rightCam, std::vector<Point> &pointCloud)
{
    cv::Mat disparity;
    computeDisparity(leftCam, rightCam, disparity);

    for (int y = 0; y < disparity.rows; y += 10)
    {
        for (int x = 0; x < disparity.cols; x += 10)
        {
            short pixel = disparity.at<short>(y, x);

            // only send pixels with disparities greater than 0
            if (pixel > 0)
            {
                Point p = convert2dPointTo3d(x, y, pixel);
                pointCloud.push_back(p);
            }
        }
    }
    // visualizePointCloud(pointCloud);
}

/*
void Vision::visualizePointCloud(const std::vector<Point>& pointCloud)
{
    // Create a PCL point cloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill the cloud with points from the input vector
    for (const auto& p : pointCloud)
    {
        cloud->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }

    // Set the number of points in the cloud
    cloud->width = cloud->points.size();
    cloud->height = 1;

    // Create a PCL cloud viewer
    pcl::visualization::CloudViewer viewer("Point Cloud Viewer");

    // Show the cloud
    viewer.showCloud(cloud);

    // Wait until the viewer is closed
    while (!viewer.wasStopped())
    {
    }
}
*/