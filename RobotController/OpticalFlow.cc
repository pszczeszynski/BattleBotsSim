#include "OpticalFlow.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/video/video.hpp>
#include <math.h>
#include <map>
#include <algorithm>
#include "TrackingUtils.h"
#include <chrono>
#include "Timer.h"

using namespace std;
using namespace cv;
#define PI 3.14159265358979323

OpticalFlow::OpticalFlow() : FRAMES_PER_TRACK(1), ct()
{
}

void OpticalFlow::InitializeMotionDetection(cv::cuda::GpuMat &img_gray)
{
    imageWidth = img_gray.size().width;
    imageHeight = img_gray.size().height;

    cv::cuda::GpuMat blank = cv::cuda::GpuMat();

    ct.ChooseNewTrackingPoints(img_gray); // droneMask
}

double OpticalFlow::CalculateAngleDelta()
{
    if (ct.corners.size() < 2)
    {
        return 0;
    }

    ValueBin angles{1};
    double MAX_ANGLE_CHANGE_PER_FRAME = 30.0 * PI / 180.0;
    double cornerSpread = ct.LargestBoundingBoxSide();
    double sumAngleChange = 0;

    int MAX_NUM_SAMPLE_POINTS = 10000000;
    double MIN_DISTANCE_TO_CONSIDER = cornerSpread * 0.25;
    int numSamplePoints = 0;
    double weightSum = 0;

    for (uint i = 0; i < ct.corners.size() - 1 && numSamplePoints < MAX_NUM_SAMPLE_POINTS; i += 1)
    {
        // don't consider crazy corners
        if (ct.corners[i].isCrazy)
        {
            continue;
        }

        for (uint j = i + 1; j < ct.corners.size() && numSamplePoints < MAX_NUM_SAMPLE_POINTS; j += 1)
        {
            // don't consider crazy corners
            if (ct.corners[j].isCrazy)
            {
                continue;
            }


            double distance1 = TrackingUtils::Distance(ct.corners[i].pl, ct.corners[j].pl);

            if (distance1 < MIN_DISTANCE_TO_CONSIDER)
            {
                continue;
            }

            double angle1 = TrackingUtils::AngleBetweenPoints(ct.corners[i].pl, ct.corners[j].pl);
            double angle2 = TrackingUtils::AngleBetweenPoints(ct.corners[i].pf, ct.corners[j].pf);
            double deltaAngle = TrackingUtils::AngleWrap(angle2 - angle1);

            if (fabs(deltaAngle) > MAX_ANGLE_CHANGE_PER_FRAME)
            {
                continue;
            }

            angles.AddValue(deltaAngle * 180.0 / PI);

            double weight = distance1;
            sumAngleChange += deltaAngle * weight;
            weightSum += weight;

            numSamplePoints++;
        }
    }

    double angleChange = angles.GetModeValue() * PI / 180.0;

    return angleChange;


    if (fabs(sumAngleChange / weightSum) > MAX_ANGLE_CHANGE_PER_FRAME)
    {
        return 0;
    }

    return sumAngleChange / weightSum;
}

double OpticalFlow::CalculateScaleDelta()
{
    if (ct.corners.size() < 2)
    {
        return 1;
    }

    const int MIN_SAMPLE_POINTS = 5;

    double sumScaleChange = 0;
    double weightSum = 0;
    int numSamplePoints = 0;

    for (uint i = 0; i < ct.corners.size() - 1; i += 1)
    {
        if (ct.corners[i].isCrazy)
        {
            continue;
        }

        for (uint j = i + 1; j < ct.corners.size(); j += 1)
        {
            if (ct.corners[j].isCrazy)
            {
                continue;
            }

            double distance1 = TrackingUtils::Distance(ct.corners[i].pl, ct.corners[j].pl);
            double distance2 = TrackingUtils::Distance(ct.corners[i].pf, ct.corners[j].pf);

            double weight = distance1;
            sumScaleChange += (distance2 / distance1) * weight;
            weightSum += weight;
            numSamplePoints++;
        }
    }

    double tentativeScaleDelta = 0;

    if (weightSum > 0 && numSamplePoints > MIN_SAMPLE_POINTS)
    {
        tentativeScaleDelta = sumScaleChange / weightSum;
    }

    if (tentativeScaleDelta > 0.8 && tentativeScaleDelta < 1.2)
    {
        return tentativeScaleDelta;
    }

    return 1;
}

cv::Point2f OpticalFlow::CalculateTranslationChangeXY(cv::Mat &drawingImage)
{
    if (ct.corners.size() < 1)
    {
        return cv::Point2f(0, 0);
    }

    ValueBin xBin;
    ValueBin yBin;

    for (int i = 0; i < ct.corners.size(); i++)
    {
        if (ct.corners[i].isCrazy)
        {
            continue;
        }

        double deltaX = ct.corners[i].GetDeltaXLastFrame(true);
        double deltaY = ct.corners[i].GetDeltaYLastFrame(true);

        xBin.AddValue(deltaX);
        yBin.AddValue(deltaY);
    }
    return cv::Point2f(xBin.GetModeValue(), yBin.GetModeValue());
}

void OpticalFlow::PerformMotionDetection(cv::cuda::GpuMat &img_gray, cv::Mat &drawingImage, bool trackTranslation, bool trackRotation, bool trackScale)
{
    Timer t;

    numFrames++;

    ct.PerformMotionDetection(img_gray, drawingImage);
    std::cout << "time for ct.PerformMotionDetection(): " << t.GetTimeMS() << std::endl;
    t.Start();

    if (trackRotation)
    {
        // Calculate angle change angle
        angleDelta = CalculateAngleDelta();
        trackedCameraAngle += angleDelta;
    }
    else
    {
        angleDelta = 0;
    }

    // trackedCameraAngle *= 0.997;

    // // now that we calculated our scale + angle delta, pass that to the corners
    // for (int i = 0; i < ct.corners.size(); i++)
    // {
    //     // for now don't pass scale
    //     ct.corners[i].SetRotAndScale(1, 0, imageWidth, imageHeight);
    // }

    if (trackScale)
    {
        scaleDelta = CalculateScaleDelta();
        trackedScale *= scaleDelta;
    }
    else
    {
        scaleDelta = 1;
    }

    // now we can set their scales
    for (int i = 0; i < ct.corners.size(); i++)
    {
        ct.corners[i].SetRotAndScale(scaleDelta, angleDelta, imageWidth, imageHeight);
    }


    if (trackTranslation)
    {
        // finally compute the translation change
        translateDelta = CalculateTranslationChangeXY(drawingImage);
    }
    else
    {
        translateDelta = cv::Point2f(0, 0);
    }

    DrawTrackerVelocityDistribution(drawingImage);

    trackedCameraPosition.x -= translateDelta.x;
    trackedCameraPosition.y -= translateDelta.y;

    ct.DrawTrackers(drawingImage, cv::Scalar(0, 255.0, 0));
    DrawVisualizerGrid(drawingImage);

    // double confidence = CalcConfidence(drawingImage);

    std::cout << "time for calculations: " << t.GetTimeMS() << std::endl;
    t.Start();

    if (numFrames % FRAMES_PER_TRACK == 0)
    {
        ct.ChooseNewTrackingPoints(img_gray);
    }

    std::cout << "time to choose new points: " << t.GetTimeMS() << std::endl;

    // if (numFrames % 2 == 0)
    //{
    //     imshow("Frame", drawingImage);
    //     waitKey(1);
    // }
}

void OpticalFlow::DrawVisualizerGrid(cv::Mat &drawingImage)
{
    // draw grid
    double gridSize = 20 * trackedScale;

    gridPosition.x += translateDelta.x * cos(trackedCameraAngle) + translateDelta.y * sin(trackedCameraAngle);
    gridPosition.y += translateDelta.y * cos(trackedCameraAngle) - translateDelta.x * sin(trackedCameraAngle);
    gridPosition = TrackingUtils::ScalePoint(gridPosition, cv::Point(0, 0), scaleDelta);
    gridPosition.x = fmod(gridPosition.x, gridSize);
    gridPosition.y = fmod(gridPosition.y, gridSize);

    double xStart = imageWidth / 2 + gridPosition.x - gridSize * (int)(imageWidth / gridSize);
    double yStart = imageHeight / 2 + gridPosition.y - gridSize * (int)(imageHeight / gridSize);

    cv::Point rotateCenter = cv::Point(imageWidth / 2, imageHeight / 2);
    for (double y = yStart; y <= imageHeight + gridSize; y += gridSize)
    {
        cv::Point2f start = cv::Point2f(-imageWidth, y);
        cv::Point2f end = cv::Point2f(2 * imageWidth, y);

        start = TrackingUtils::RotatePoint(start, rotateCenter, trackedCameraAngle);
        end = TrackingUtils::RotatePoint(end, rotateCenter, trackedCameraAngle);

        cv::line(drawingImage, start, end, cv::Scalar(0, 255, 255, 1), 1);
    }

    for (double x = xStart; x <= imageWidth + gridSize; x += gridSize)
    {
        cv::Point2f start = cv::Point2f(x, -imageHeight);
        cv::Point2f end = cv::Point2f(x, imageHeight * 2);

        start = TrackingUtils::RotatePoint(start, rotateCenter, trackedCameraAngle);
        end = TrackingUtils::RotatePoint(end, rotateCenter, trackedCameraAngle);

        cv::line(drawingImage, start, end, cv::Scalar(0, 255, 255, 1), 1);
    }
}

/**
 * Returns a double from 0 to 1 indicating our tracking confidence level.
 */
double OpticalFlow::CalcConfidence(cv::Mat &drawingImage)
{
    // 1. Use the number of corners
    double numTrackersScore = ct.corners.size() / 100.0;
    if (numTrackersScore > 1.0)
    {
        numTrackersScore = 1.0;
    }
    if (numTrackersScore == 0.0)
    {
        return 0;
    }

    // 2. Use the err[] from ct
    // skip for now

    // 3. Use the spread of the points
    int numPointsCloseToLeftEdge = 0;
    int numPointsCloseToMiddle = 0;
    int numPointsCloseToRightEdge = 0;

    for (cv::Point2f p : ct.pl)
    {
        if (p.x < imageWidth * 0.33)
        {
            numPointsCloseToLeftEdge++;
        }
        else if (p.x > imageWidth * 0.66)
        {
            numPointsCloseToRightEdge++;
        }
        else
        {
            numPointsCloseToMiddle++;
        }
    }

    double pointsCloseToMiddleScore = (double)numPointsCloseToLeftEdge / (ct.pl.size() / 3.0);
    if (pointsCloseToMiddleScore > 1.0)
    {
        pointsCloseToMiddleScore = 1.0;
    }
    double pointsCloseToLeftEdgeScore = (double)numPointsCloseToLeftEdge / (ct.pl.size() / 3.0);
    if (pointsCloseToLeftEdgeScore > 1.0)
    {
        pointsCloseToLeftEdgeScore = 1.0;
    }
    double pointsCloseToRightEdgeScore = (double)numPointsCloseToRightEdge / (ct.pl.size() / 3.0);
    if (pointsCloseToRightEdgeScore > 1.0)
    {
        pointsCloseToRightEdgeScore = 1.0;
    }
    double pointSpreadScore = (pointsCloseToMiddleScore + pointsCloseToLeftEdgeScore + pointsCloseToRightEdgeScore) / 3.0;

    // 4. Use the velocity distributions -> if all moving roughly the same amount, high confidence

    // double velDistributionScoreX = (double) xBins.GetCountForValue(xBins.GetModeValue()) / ct.pl.size();
    // double velDistributionScoreY = (double) yBins.GetCountForValue(yBins.GetModeValue()) / ct.pl.size();
    // double velDistributionScore = (velDistributionScoreX + velDistributionScoreY) / 2.0;

    cv::rectangle(drawingImage, cv::Rect(150, 150 - 100 * numTrackersScore, 25, 100 * numTrackersScore), cv::Scalar(0, numTrackersScore * 255.0, 255.0 - numTrackersScore * 255.0), -1);
    cv::rectangle(drawingImage, cv::Rect(200, 150 - 100 * pointSpreadScore, 25, 100 * pointSpreadScore), cv::Scalar(0, pointSpreadScore * 255.0, 255.0 - pointSpreadScore * 255.0), -1);
    // cv::rectangle(drawingImage, cv::Rect(250, 150 - 100 * velDistributionScore, 25, 100 * velDistributionScore), cv::Scalar(0, velDistributionScore * 255.0, 255.0 - velDistributionScore * 255.0), -1);

    // std::cout << "mode x velocity: " << xBins.GetModeValue() << std::endl;
    // std::cout << "mode x velocity count: " << xBins.GetCountForValue(xBins.GetModeValue()) << std::endl;
    // std::cout << "mode y velocity: " << yBins.GetModeValue() << std::endl;
    // std::cout << "mode y velocity count: " << yBins.GetCountForValue(yBins.GetModeValue()) << std::endl;

    return (numTrackersScore + pointSpreadScore) / 2.0; //  + velDistributionScore
}

double OpticalFlow::GetXPosition()
{
    return trackedCameraPosition.x;
}

double OpticalFlow::GetYPosition()
{
    return trackedCameraPosition.y;
}

double OpticalFlow::GetScale()
{
    return trackedScale;
}

double OpticalFlow::GetRotation()
{
    return trackedCameraAngle;
}

void OpticalFlow::SetRotation(double rotation)
{
    trackedCameraAngle = rotation;
}

void OpticalFlow::DrawTrackerVelocityDistribution(cv::Mat &drawingImage)
{
    ValueBin velocities(10);
    for (uint i = 0; i < ct.corners.size(); i++)
    {
        double deltaXFloat = ct.corners[i].pf_shifted.x - ct.corners[i].pi.x;
        double deltaYFloat = ct.corners[i].pf_shifted.y - ct.corners[i].pi.y;

        velocities.AddValue(sqrt(pow(deltaXFloat, 2) + pow(deltaYFloat, 2)));
    }

    std::cout << "bins size: " << velocities.GetSize() << std::endl;
    std::cout << "bins getting count for value 0 it is: " << velocities.GetCountForValue(0) << std::endl;

    double averageValue = velocities.GetAverageValue();
    std::cout << "bins average value: " << averageValue << std::endl;
    int totalCount = 0;
    int maxVel = 30;
    for (double i = 0; i < maxVel; i += 1.0 / 10.0)
    {
        int thisCount = velocities.GetCountForValue((i - maxVel / 2) + averageValue);
        cv::rectangle(drawingImage, cv::Rect(drawingImage.size().width * (double)i / maxVel, 0, drawingImage.size().width / (maxVel * 10.0), thisCount), cv::Scalar(0, 0, 255), -1);

        totalCount += thisCount;
    }

    if (totalCount < ct.corners.size())
    {
        std::cout << "COUNT MISMATCH! Count from bins: " << totalCount << " count from ct.pl: " << ct.pl.size() << std::endl;
    }
}
