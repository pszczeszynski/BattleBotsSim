#include "Vision.h"
#include "MathUtils.h"
#include "RobotStateParser.h"

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include "PathFinder.h"
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/flann.hpp>

// GLint doesn't have different sizes on different compilers whereas int does
const GLint POINT_CLOUD_WINDOW_WIDTH = 1920, POINT_CLOUD_WINDOW_HEIGHT = 1080;
#define WIDTH 1280
#define HEIGHT 720
#define DISPARITY_SCALAR 4500.0

const int NUM_DISPARITIES = 256;
const int MIN_DISPARITY = 0;
const int BLOCK_SIZE = 3;
const int MAX_POINT_CLOUD_SIZE = 500000;
// camera properties obtained from unity
#define FOV_X TO_RAD * 100
#define FOV_Y TO_RAD * 60

Vision::Vision(ICameraReceiver &overheadCamL, ICameraReceiver &overheadCamR)
    : overheadCamL(overheadCamL),
      overheadCamR(overheadCamR),
      opticalFlow(),
      opponentOpticalFlow()
{

    // Setup StereoSGBM
    const int channels = 1;
    stereoSGMMain = cv::cuda::createStereoSGM(MIN_DISPARITY, NUM_DISPARITIES, 9, 60, 15, 3);
    stereoSGBMMainCPU = cv::StereoSGBM::create(MIN_DISPARITY, NUM_DISPARITIES, BLOCK_SIZE);

    // Define the stereo matching method and parameters
    stereoSGMMain->setBlockSize(BLOCK_SIZE);
    // stereoSGMMain->setBlockSize(BLOCK_SIZE);
    stereoSGMMain->setP1(1 * channels * 3 * 3);
    stereoSGMMain->setP2(2 * channels * 3 * 3); // increasing makes sort of smoother -> more blobby

    // add 2 robot trackers
    robotTrackers.push_back(RobotTracker(cv::Point2f(0,0)));
    robotTrackers.push_back(RobotTracker(cv::Point2f(10000, 10000)));

    gameLoopThread = new std::thread([this]()
                                     {
        // setup the window
        const Engine::WindowSettings myWindowSettings = {POINT_CLOUD_WINDOW_WIDTH, POINT_CLOUD_WINDOW_HEIGHT};
        Engine::Window myWindow = Engine::Window(myWindowSettings);
        // show the window
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
        myWindow.startLoop(); });
}

// void Vision::performOpticalFlow()
// {
//     cv::Mat frame = overheadCam.getFrame();
//     cv::Mat drawingImage = frame.clone();

//     // upload to gpu + convert to grayscale
//     cv::cuda::GpuMat gpuFrame(frame);
//     cv::cuda::GpuMat gpuGrayFrame;

//     cv::cuda::cvtColor(gpuFrame, gpuGrayFrame, cv::COLOR_BGR2GRAY);


//     if (!isInitialized)
//     {
//         opticalFlow.InitializeMotionDetection(gpuGrayFrame);
//         isInitialized = true;
//     }
//     else
//     {
//         opticalFlow.PerformMotionDetection(gpuGrayFrame, drawingImage);
//         cv::imshow("optical flow", drawingImage);
//     }
// }

bool areMatsEqual(const cv::Mat &mat1, const cv::Mat &mat2)
{
    if (mat1.size() != mat2.size() || mat1.type() != mat2.type())
    {
        // Mats have different sizes or types
        return false;
    }

    // Compute the absolute difference between the current frame and the previous frame
    cv::Mat diff;
    cv::absdiff(mat1, mat2, diff);

    // Convert the difference to grayscale
    cv::Mat grayDiff;
    cv::cvtColor(diff, grayDiff, cv::COLOR_BGR2GRAY);

    return cv::countNonZero(grayDiff) == 0;
}

void equalizeHistogram3Channels(cv::Mat& src, cv::Mat& dst)
{
    // std::vector<cv::Mat> channels;
    // cv::split(src, channels);
    // for (int i = 0; i < 3; i++)
    // {
    //     cv::equalizeHist(channels[i], channels[i]);
    // }
    // cv::merge(channels, dst);


    // this version just normalizes each channel
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    for (int i = 0; i < 3; i++)
    {
        cv::normalize(channels[i], channels[i], 0, 255, cv::NORM_MINMAX);
    }
    cv::merge(channels, dst);

}

void alignImages(const cv::Mat& src1, const cv::Mat& src2, cv::Mat& dst1, cv::Mat& dst2)
{
    // Set the maximum shift range
    int maxShift = src1.rows / 10;

    // Define the subset region for comparison
    int subsetY = maxShift;
    int subsetHeight = src1.rows - maxShift * 2;

    // Initialize variables to track the best alignment
    int bestShift = 0;
    double bestDiff = std::numeric_limits<double>::max();

    // Iterate over the shift range and find the best alignment
    for (int shift = -maxShift; shift <= maxShift; ++shift)
    {
        // Extract the subsets from each image
        cv::Rect roi(0, subsetY, src1.cols, subsetHeight);
        cv::Mat subset1 = src1(roi);
        cv::Mat subset2 = src2(roi + cv::Point(0, shift));

        // Compute the absolute difference between the subsets
        double diffSum = cv::norm(subset1, subset2, cv::NORM_L1);

        // Update the best alignment if the difference is smaller
        if (diffSum < bestDiff)
        {
            bestDiff = diffSum;
            bestShift = shift;
        }
    }

    // Adjust the subset region with the best alignment
    cv::Rect roi(0, subsetY, src1.cols, subsetHeight - bestShift);

    // Extract the aligned subsets
    cv::Mat alignedSubset1 = src1(roi).clone();
    cv::Mat alignedSubset2 = src2(roi + cv::Point(0, bestShift)).clone();

    // Create the aligned destination images
    dst1 = src1(roi);
    dst2 = src2(roi + cv::Point(0, bestShift));

}



void correctStereoColor(cv::Mat& leftImage, cv::Mat& rightImage, cv::Mat& leftImageCorrected, cv::Mat& rightImageCorrected) {
    // Split the input images into individual channels
    std::vector<cv::Mat> leftChannels, rightChannels;
    cv::split(leftImage, leftChannels);
    cv::split(rightImage, rightChannels);

    // Calculate the mean of each channel
    cv::Scalar leftMean = cv::mean(leftImage);
    cv::Scalar rightMean = cv::mean(rightImage);

    // Calculate the average brightness for each channel
    double leftBrightnessB = leftMean[0];
    double leftBrightnessG = leftMean[1];
    double leftBrightnessR = leftMean[2];

    double rightBrightnessB = rightMean[0];
    double rightBrightnessG = rightMean[1];
    double rightBrightnessR = rightMean[2];

    // Calculate the scaling ratios for each channel
    double ratioB = leftBrightnessB / rightBrightnessB;
    double ratioG = leftBrightnessG / rightBrightnessG;
    double ratioR = leftBrightnessR / rightBrightnessR;

    // Scale the right image channels independently
    cv::multiply(rightChannels[0], ratioB, rightChannels[0]);
    cv::multiply(rightChannels[1], ratioG, rightChannels[1]);
    cv::multiply(rightChannels[2], ratioR, rightChannels[2]);

    // Merge the corrected channels back into images
    cv::merge(leftChannels, leftImageCorrected);
    cv::merge(rightChannels, rightImageCorrected);
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

void Vision::DrawRobots()
{
    cv::Mat frameWithCircles = currFrameL.clone();

    const cv::Point2f robotPosition = GetRobotPosition();
    const cv::Point2f opponentPosition = GetOpponentPosition();
    const double robotAngle = GetRobotAngle();
    const double opponentAngle = GetOpponentAngle();

    // draw lines starting at the center of each robot showing their angles
    const double RADIUS = 30;

    cv::circle(frameWithCircles, robotPosition, RADIUS, cv::Scalar(255,0,0), 4);
    cv::circle(frameWithCircles, opponentPosition, RADIUS, cv::Scalar(0,0,255), 4);

    cv::line(frameWithCircles, robotPosition, robotPosition + cv::Point2f(cos(robotAngle) * RADIUS, sin(robotAngle) * RADIUS), cv::Scalar(255,0,0), 4);
    cv::line(frameWithCircles, opponentPosition, opponentPosition + cv::Point2f(cos(opponentAngle) * RADIUS, sin(opponentAngle) * RADIUS), cv::Scalar(0,0,255), 4);

    cv::imshow("circles", frameWithCircles);
}

void Vision::DetectRotation(cv::Mat& canny)
{
    const cv::Point2f robotPosition = GetRobotPosition();

    int CROP_RADIUS = 60;
    // crop us out
    cv::Mat cropped = canny(cv::Rect(robotPosition.x - CROP_RADIUS, robotPosition.y - CROP_RADIUS, CROP_RADIUS * 2, CROP_RADIUS * 2));

    cv::imshow("cropped", cropped);
    // get average angle of canny lines
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(cropped, lines, 1, CV_PI/180, 10, 3, 0);


    cv::Mat linesImage = cv::Mat::zeros(cropped.size(), CV_8UC3);
    ValueBin rotations(3);
    // show the vector of lines in the image
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        cv::line(linesImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,255), 1, cv::LINE_AA);

        double degrees = TO_DEG * atan2(l[3] - l[1], l[2] - l[0]);
        while (degrees < 0)
        {
            degrees += 180;
        }
        if (degrees == 0)
        {
            continue;
        }
        // take mode pi / 2
        double mode = fmod(degrees, 180);
    
        rotations.AddValue(mode);
    }


    // get mode of rotations
    double mode = rotations.GetModeValue();
    // convert to radians
    double radians = mode * TO_RAD;
    // draw arrow at middle of cropped
    cv::Point2f center = cv::Point2f(cropped.cols / 2, cropped.rows / 2);
    cv::Point2f end = center + cv::Point2f(cos(radians) * 100, sin(radians) * 100);
    cv::arrowedLine(linesImage, center, end, cv::Scalar(0, 255, 0), 3);






    cv::imshow("lines", linesImage);

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

// void Vision::runPipeline()
// {
//     overheadCamL.getFrame(currFrameL);

//     // scale to 1280 by 720
//     cv::resize(currFrameL, currFrameL, cv::Size(WIDTH, HEIGHT));


//     // Skip the first frame or if the current frame is the same as the previous frame
//     if (previousFrameL.empty() || areMatsEqual(currFrameL, previousFrameL))
//     {
//         previousFrameL = currFrameL.clone();
//         return;
//     }

//     convertToBirdsEyeView(currFrameL, currFrameL);

//     // find the opponent
//     locateRobots(currFrameL, previousFrameL);
//     previousFrameL = currFrameL.clone();

//     // DrawRobots();


//     cv::Mat linesImage;
//     // detect the lines in the image
//     detectLines(currFrameL, linesImage);

//     // DetectRotation(linesImage);
//     DetectBottomLineAndShowImage(linesImage);
// }

// // Function to build a kd-tree from an array of cv::Point3f points
// cv::flann::Index buildKDTree(const std::vector<cv::Point3f> &points)
// {
//     cv::Mat data(points.size(), 3, CV_32F);
//     for (size_t i = 0; i < points.size(); ++i)
//     {
//         data.at<float>(i, 0) = points[i].x;
//         data.at<float>(i, 1) = points[i].y;
//         data.at<float>(i, 2) = points[i].z;
//     }

//     cv::flann::Index index(data, cv::flann::KDTreeIndexParams());
//     index.build(data, cv::flann::KDTreeIndexParams());
//     return index;
// }

#define OVERHEAD_WIDTH 1280
#define OVERHEAD_HEIGHT 1280

void convertPointCloudToOverhead(std::vector<cv::Point3f> pointCloud, std::vector<cv::Vec3b> colors, cv::Mat& dstOverhead)
{
    // init to 1280 by 1280
    dstOverhead = cv::Mat::zeros(cv::Size(OVERHEAD_WIDTH, OVERHEAD_HEIGHT), CV_32FC3);

    // the total weight at each point
    cv::Mat colorWeightSums = cv::Mat::zeros(cv::Size(OVERHEAD_WIDTH, OVERHEAD_HEIGHT), CV_32FC3);

    // for each point, convert to overhead
    for (int i = pointCloud.size() - 1; i >= 0; i--)
    {
        cv::Point3f point = pointCloud[i];
        cv::Vec3b color = colors[i];
        if (point.y > -0.6)
        {
            continue;
        }
        // align
        point += cv::Point3f(1, 0, 0.2);
        point *= 500;
        

        // check if point is in bounds
        if (point.x < 0 || point.x >= OVERHEAD_WIDTH || point.z < 0 || point.z >= OVERHEAD_HEIGHT)
        {
            continue;
        }

        int radius = 3;
        for (int rel_x = -radius; rel_x <= radius; rel_x++)
        {
            for (int rel_y = -radius; rel_y <= radius; rel_y++)
            {
                int x = point.x + rel_x;
                int y = point.z + rel_y;

                if (x < 0 || x >= OVERHEAD_WIDTH || y < 0 || y >= OVERHEAD_HEIGHT)
                {
                    continue;
                }


                double weight = 1 / (pow(cv::norm(cv::Point(rel_x, rel_y)), 2) + 0.3);
                // weight *= (point.y / 500 - (-1.5)) / 0.3;
                // add color to overhead
                dstOverhead.at<cv::Vec3f>(cv::Point(x, y)) += color * weight;
                colorWeightSums.at<cv::Vec3f>(cv::Point(x, y)) += cv::Vec3f(1, 1, 1) * weight;
            }
        }


        if (i == 0)
        {
            std::cout << "first color: " << color << std::endl;
        }
    }

    // divide by weight sums
    dstOverhead /= colorWeightSums;

    // convert to 8 bit
    dstOverhead.convertTo(dstOverhead, CV_8UC3);

    cv::imshow("overhead", dstOverhead);
}

void Vision::runPipeline()
{
    overheadCamL.getFrame(currFrameL);
    overheadCamR.getFrame(currFrameR);

    std::cout << "currFrameL.size():  " << currFrameL.size() << std::endl;

    // alignImages(currFrameL, currFrameR, currFrameL, currFrameR);

    // compute individual point clouds
    std::vector<cv::Vec3b> colors = {};
    std::vector<cv::Point3f> pointCloud = {};
    compute3dPointCloud(currFrameL, currFrameR, pointCloud, colors);
    cv::Mat overheadView;
    convertPointCloudToOverhead(pointCloud, colors, overheadView);

    // if gameloop created, submit vertices + opponent position
    if (pGameLoop != nullptr)
    {
        if (pointCloud.size() > 0)
        {
            pGameLoop->SetPointCloudVerts(pointCloud, colors);
        }
        pGameLoop->SetOpponentPosition({0, 0, 0});
    }
}

const cv::Mat& Vision::GetBirdsEyeImageL()
{
    return previousFrameL;
}

const cv::Mat& Vision::GetBirdsEyeImageR()
{
    return previousFrameR;
}

void Vision::locateRobots(cv::Mat& frame, cv::Mat& previousFrameL)
{
    const cv::Size BLUR_SIZE = cv::Size(14,14);

    const float MIN_AREA = 1200;

    cv::Point2f center = cv::Point2f(0,0);

    // Compute the absolute difference between the current frame and the previous frame
    cv::Mat diff;
    cv::absdiff(previousFrameL, frame, diff);

    // Convert the difference to grayscale
    cv::Mat grayDiff;
    cv::cvtColor(diff, grayDiff, cv::COLOR_BGR2GRAY);

    // Convert the difference to a binary image with a certain threshold    
    cv::Mat thresholdImg;
    cv::threshold(grayDiff, thresholdImg, 50, 255, cv::THRESH_BINARY);

    // blurr and re-thresh to make it more leanient
    cv::blur(thresholdImg, thresholdImg, BLUR_SIZE);
    cv::threshold(thresholdImg, thresholdImg, 25, 255, cv::THRESH_BINARY);

    // find big blobs in the image using a blob detector

    // iterate through every pixel in the image and find the largest blob
    std::vector<cv::Rect> potentialRobots = {};
    for (int y = 0; y < thresholdImg.rows; y += 10)
    {
        for (int x = 0; x < thresholdImg.cols; x += 10)
        {
            // if this pixel is white, then it is part of a blob
            if (thresholdImg.at<uchar>(y, x) == 255)
            {
                // flood fill the blob
                cv::Rect rect;
                // flood fill but don't change the image
                cv::floodFill(thresholdImg, cv::Point(x, y), cv::Scalar(100), &rect);

                // if the blob is larger than the previous largest blob, then update the largest blob
                if (rect.area() > MIN_AREA)
                {
                    // add the rect to the list of potential robots
                    potentialRobots.push_back(rect);
                }
            }
        }
    }

    // for each robot, find the EXACT center of the robot by counting the white pixels in the blob and averaging them
    std::vector<cv::Point2f> robotCenters;
    std::vector<MotionBlob> motionBlobs;
    for (const cv::Rect &rect : potentialRobots)
    {
        // find the average of all the white pixels in the blob
        int numWhitePixels = 0;
        cv::Point2f averageWhitePixel = cv::Point2f(0,0);
        for (int y = rect.y; y < rect.y + rect.height; y++)
        {
            for (int x = rect.x; x < rect.x + rect.width; x++)
            {
                // if this pixel is white, then add it to the average
                if (thresholdImg.at<uchar>(y,x) > 0)
                {
                    averageWhitePixel += cv::Point2f(x,y);
                    numWhitePixels++;
                }
            }
        }
        // divide by the number of white pixels to get the average
        averageWhitePixel /= numWhitePixels;

        // add the average to the list of robot centers
        motionBlobs.emplace_back(MotionBlob{rect, averageWhitePixel, &frame});
    }

    // update the robot trackers
    updateRobotTrackers(motionBlobs, frame);
}

cv::Point2f Vision::GetRobotPosition()
{
    return robotTrackers[0].getPosition();
}

double Vision::GetRobotAngle()
{
    return robotTrackers[0].getAngle();
}

cv::Point2f Vision::GetOpponentPosition()
{
    return robotTrackers[1].getPosition();
}

double Vision::GetOpponentAngle()
{
    return robotTrackers[1].getAngle();
}

void Vision::updateRobotTrackers(std::vector<MotionBlob>& centers, cv::Mat& frame)
{
    // minimize cost of updating
    if (centers.empty()) return;

    double COST_THRESHOLD = 0.15; // above this cost means we're not updating
    std::vector<bool> updatedTrackers = {};
    // fill updatedTrackers with false
    for (int i = 0; i < robotTrackers.size(); i++) updatedTrackers.push_back(false);

    if (centers.size() == 1)
    {
        double cost1 = robotTrackers[0].getCostOfUpdating(centers[0]);
        double cost2 = robotTrackers[1].getCostOfUpdating(centers[0]);
        // std::cout << "cost1: " << cost1 << std::endl;

        // assume the robot is us if we're moving
        if (cost1 < COST_THRESHOLD)
        {
            robotTrackers[0].update(centers[0], frame);
            updatedTrackers[0] = true;
        }
    }
    else
    {
        double cost00 = robotTrackers[0].getCostOfUpdating(centers[0]);
        double cost01 = robotTrackers[0].getCostOfUpdating(centers[1]);
        double cost10 = robotTrackers[1].getCostOfUpdating(centers[0]);
        double cost11 = robotTrackers[1].getCostOfUpdating(centers[1]);

        if (cost00 + cost11 < cost01 + cost10)
        {
            if (cost00 < COST_THRESHOLD)
            {
                robotTrackers[0].update(centers[0], frame);
                updatedTrackers[0] = true;
            }

            if (cost11 < COST_THRESHOLD)
            {
                robotTrackers[1].update(centers[1], frame);
                updatedTrackers[1] = true;
            }
        }
        else
        {
            if (cost01 < COST_THRESHOLD)
            {
                robotTrackers[0].update(centers[1], frame);
                updatedTrackers[0] = true;
            }

            if (cost10 < COST_THRESHOLD)
            {
                robotTrackers[1].update(centers[0], frame);
                updatedTrackers[1] = true;
            }
        }
    }

    // for each tracker that wasn't updated, mark it as invalid
    for (int i = 0; i < robotTrackers.size(); i++)
    {
        if (!updatedTrackers[i])
        {
            robotTrackers[i].invalidate();
        }
    }
}

void Vision::convertToBirdsEyeView(cv::Mat& frame, cv::Mat& dst)
{
    // Define the four corners of the region of interest (ROI) in the input image
    cv::Point2f srcPoints[4] = {
        cv::Point2f(0, 0),    // Top-left corner
        cv::Point2f(frame.cols, 0),    // Top-right corner
        cv::Point2f(frame.cols, frame.rows),    // Bottom-right corner
        cv::Point2f(0, frame.rows)     // Bottom-left corner
    };

    // Define the dimensions of the bird's-eye view output image
    int outputWidth = frame.cols;
    int outputHeight = frame.rows;

    // Define the four corners of the bird's-eye view output image
    cv::Point2f dstPoints[4] = {
        cv::Point2f(0, 0),                      // Top-left corner
        cv::Point2f(outputWidth, 0),        // Top-right corner
        cv::Point2f(outputWidth * 0.65, outputHeight),  // Bottom-right corner
        cv::Point2f(outputWidth * 0.35, outputHeight)        // Bottom-left corner
    };

    // Compute the transformation matrix
    cv::Mat transformationMatrix = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // Apply the perspective transformation
    warpPerspective(frame, dst, transformationMatrix, cv::Size(outputWidth, outputHeight));
}

cv::Point3f rotate3dPoint(cv::Point3f p, double angle_rad)
{
    p.y = p.y * cos(angle_rad) - p.z * sin(angle_rad);
    p.z = p.y * sin(angle_rad) + p.z * cos(angle_rad);
    return p;
}

#define CAMERA_ANGLE_TOWARDS_GROUND_RAD 40.767 * TO_RAD
cv::Point3f Vision::convert2dPointTo3d(int x, int y, short disparity)
{
    // calc normalized positions
    // 1 means fully to the top of the image or to the right
    // -1 means fully to the bottom of the image or to the left
    float xNormalized = (x - (WIDTH / 2.0)) / (WIDTH / 2.0);
    float yNormalized = -(y - (HEIGHT / 2.0)) / (HEIGHT / 2.0);
    float zPos = DISPARITY_SCALAR / disparity;

    cv::Point3f point{xNormalized, yNormalized, zPos};

    point.x = xNormalized * sin(FOV_X / 2) * zPos;
    point.y = yNormalized * sin(FOV_Y / 2) * zPos;


    // rotate point 45 degrees on the x axis
    point = rotate3dPoint(point, CAMERA_ANGLE_TOWARDS_GROUND_RAD);
    return point;
}

void extendImageLeftSide(const cv::Mat &src, cv::Mat &dst, int width)
{
    // Create an image with the added black rectangle
    dst = cv::Mat(src.rows, src.cols + width, CV_8UC3, cv::Scalar(0, 0, 0));
    src.copyTo(dst(cv::Rect(width, 0, src.cols, src.rows)));
}

void Vision::computeDisparity(const cv::Mat &left, const cv::Mat &right, cv::Mat &disparity, cv::Mat &disparityNormalized)
{
    cv::Mat leftCrop = left;
    cv::Mat rightCrop = right;

    // extend the left side of the images with black. This is because the greater
    // NUM_DISPARITIES, the greater crop on the left side the disparity map will have.
    cv::Mat leftCropExtended;
    extendImageLeftSide(leftCrop, leftCropExtended, NUM_DISPARITIES);
    cv::Mat rightCropExtended;
    extendImageLeftSide(rightCrop, rightCropExtended, NUM_DISPARITIES);

    cv::Mat leftChannels[3], rightChannels[3];
    cv::split(leftCropExtended, leftChannels);
    cv::split(rightCropExtended, rightChannels);

    cv::cuda::GpuMat d_disparityChannels[3];
    cv::Mat disparityChannels[3];

    // Compute the disparity map for each color channel
    for (int i = 0; i < 3; i++)
    {
        // stereoSGBMMainCPU->compute(leftChannels[i], rightChannels[i], disparityChannels[i]);
        cv::cuda::GpuMat d_leftChannel(leftChannels[i]);
        cv::cuda::GpuMat d_rightChannel(rightChannels[i]);
        stereoSGMMain->compute(d_leftChannel, d_rightChannel, d_disparityChannels[i]);
        d_leftChannel.release();
        d_rightChannel.release();
    }
    // upload the disparity channels to the GPU
    // for (int i = 0; i < 3; i++)
    // {
    //     d_disparityChannels[i].upload(disparityChannels[i]);
    //     disparityChannels[i].release();
    // }

    // Max the disparity maps for each color channel
    // Find the maximum disparity value for each pixel across all channels
    cv::cuda::GpuMat d_disparity_max(d_disparityChannels[0].size(), d_disparityChannels[0].type(), cv::Scalar(0));

    for (int i = 0; i < 1; i++)
    {
        cv::cuda::max(d_disparity_max, d_disparityChannels[i], d_disparity_max);
        d_disparityChannels[i].release();
    }

    d_disparity_max.download(disparity);
    d_disparity_max.release();


    // // crop the disparity 3 pixels each side
    // disparity = disparity(cv::Rect(3, 3, disparity.cols - 6, disparity.rows - 6));

    // Crop the left side of the image
    disparity = disparity(cv::Rect(NUM_DISPARITIES, 0, disparity.cols - NUM_DISPARITIES, disparity.rows));

    // add shift to the disparity at every pixel
    // cv::add(disparity, cv::Scalar(shift * 5), disparity);
    // medianBlur(disparity, disparity, 5);


    // // blur the disparity map
    // cv::GaussianBlur(disparity, disparity, cv::Size(5, 5), 0, 0);

    // Normalize the disparity map
    cv::normalize(disparity, disparityNormalized, 0, 255, cv::NORM_MINMAX, CV_8U);

    // disparityNormalized = 255 - disparityNormalized;
}

void Vision::compute3dPointCloud(cv::Mat &leftCam, cv::Mat &rightCam,
                                 std::vector<cv::Point3f> &pointCloud, std::vector<cv::Vec3b> &colors)
{
    cv::Mat disparity;
    cv::Mat disparityNormalized;
    computeDisparity(leftCam, rightCam, disparity, disparityNormalized);

    // // normalize, the output should range from 0 to 255
    // cv::normalize(disparity, disparity, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imshow("disparity", disparityNormalized);
    int size = 1;
    ValueBin b = ValueBin{100};

    for (int y = size; y < disparity.rows - size; y += size)
    {
        for (int x = size; x < disparity.cols - size; x += size)
        {
            // b.Clear();
            // for (int j = -size; j <= size; j++)
            // {
            //     for (int i = -size; i <= size; i++)
            //     {
            //         short disp = disparity.at<short>(y + j, x + i);
            //         cv::Vec3b color = leftCam.at<cv::Vec3b>(y + j, x + i);
            //         bool isWhite = color[0] > 150 && color[1] > 150 && color[2] > 150;

            //         if (disp == 0 || isWhite)
            //         {
            //             continue;
            //         }
            //         b.AddValue(disp);
            //     }
            // }

            // if (b.GetSize() == 0)
            // {
            //     continue;
            // }
            // short max = b.GetModeValue();
            short max = disparity.at<short>(y, x);
            if (max == 0)
            {
                continue;
            }

            cv::Vec3b color = leftCam.at<cv::Vec3b>(y, x);


            // only send pixels with disparities greater than 0
            cv::Point3f p = cv::Point3f{convert2dPointTo3d(x, y, max)};

            if (p.y < -1.155 || p.y > -0.5)
            {
                continue;
                // // set color to red
                // color = cv::Vec3b{0, 0, 255};
            }

            // p.z = 5;
            // if (p.y > 5 || cv::norm(p) > 16 || p.y < -0.5)
            // {
            //     continue;
            // }
            colors.push_back(color);
            pointCloud.push_back(p);

            // if (pointCloud.size() > MAX_POINT_CLOUD_SIZE)
            // {
            //     break;
            // }
        }
        // std::cout << "finished row #" << y << std::endl;
    }
    // leftCam.release();
    // rightCam.release();
}
