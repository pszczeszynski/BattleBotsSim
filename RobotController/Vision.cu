#include "Vision.h"
#include "MathUtils.h"
#include "RobotStateParser.h"

#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudastereo.hpp>

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include "Graphics/GameLoop.h"

// GLint doesn't have different sizes on different compilers whereas int does
const GLint POINT_CLOUD_WINDOW_WIDTH = 1920, POINT_CLOUD_WINDOW_HEIGHT = 1080;

#define WIDTH 640
#define HEIGHT 480

#define HEIGHT_DISPARITY WIDTH * 0.6

#define DISPARITY_SCALAR 3500.0

// camera properties obtained from unity
#define FOV_X TO_RAD * 75.18 * 2
#define FOV_Y TO_RAD * 60.0 * 2

const int NUM_DISPARITIES = 256;
const int MIN_DISPARITY = 0;
const int BLOCK_SIZE = 5;
const int MAX_POINT_CLOUD_SIZE = 50000;

Vision::Vision(CameraReceiver &cameraTL, CameraReceiver &cameraTR, CameraReceiver &cameraBL, CameraReceiver &cameraBR,
               CameraReceiver &cameraLL, CameraReceiver &cameraLR, CameraReceiver &cameraRL, CameraReceiver &cameraRR)
    : cameraTL(cameraTL),
      cameraTR(cameraTR),
      cameraBL(cameraBL),
      cameraBR(cameraBR),
      cameraLL(cameraLL),
      cameraLR(cameraLR),
      cameraRL(cameraRL),
      cameraRR(cameraRR)
{
    // Setup StereoSGBM
    const int channels = 1;
    stereoSGMMain = cv::cuda::createStereoSGM(MIN_DISPARITY, NUM_DISPARITIES, 18, 120, 5, 3);
    // Define the stereo matching method and parameters
    stereoSGMMain->setBlockSize(BLOCK_SIZE);
    // stereoSGMMain->setBlockSize(BLOCK_SIZE);
    stereoSGMMain->setP1(2 * channels * 3 * 3);
    stereoSGMMain->setP2(4 * channels * 3 * 3); // increasing makes sortof smoother -> more blobby

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

    cv::Mat leftChannels[3], rightChannels[3];
    cv::split(leftCropExtended, leftChannels);
    cv::split(rightCropExtended, rightChannels);

    cv::cuda::GpuMat d_disparityChannels[3];

    // Compute the disparity map for each color channel
    for (int i = 0; i < 3; i++)
    {
        cv::cuda::GpuMat d_leftChannel(leftChannels[i]);
        cv::cuda::GpuMat d_rightChannel(rightChannels[i]);
        stereoSGMMain->compute(d_leftChannel, d_rightChannel, d_disparityChannels[i]);
        d_leftChannel.release();
        d_rightChannel.release();
    }

    // Max the disparity maps for each color channel
    // Find the maximum disparity value for each pixel across all channels
    cv::cuda::GpuMat d_disparity_max(d_disparityChannels[0].size(), d_disparityChannels[0].type(), cv::Scalar(0));
    for (int i = 0; i < 3; i++)
    {
        cv::cuda::max(d_disparity_max, d_disparityChannels[i], d_disparity_max);
        d_disparityChannels[i].release();
    }

    d_disparity_max.download(disparity);
    d_disparity_max.release();

    // Crop the left side of the image
    disparity = disparity(cv::Rect(NUM_DISPARITIES, 0, disparity.cols - NUM_DISPARITIES, disparity.rows));
    // medianBlur(disparity, disparity, 5);

    cv::Mat disparityNormalized;
    // Normalize the disparity map
    cv::normalize(disparity, disparityNormalized, 0, 255, cv::NORM_MINMAX, CV_8U);

    disparityNormalized = 255 - disparityNormalized;
}

cv::Point3f Vision::convert2dPointTo3d(int x, int y, short disparity)
{
    // calc normalized positions
    // 1 means fully to the top of the image or to the right
    // -1 means fully to the bottom of the image or to the left
    float xNormalized = (x - (WIDTH / 2.0)) / (WIDTH / 2.0);
    float yNormalized = -(y - (HEIGHT / 2.0)) / (HEIGHT / 2.0);
    float zPos = DISPARITY_SCALAR / (disparity);

    cv::Point3f point{xNormalized, yNormalized, zPos * 1.3f};

    point.x = xNormalized * sin(FOV_X / 2) * zPos;
    point.y = yNormalized * sin(FOV_Y / 2) * zPos;

    return point;
}

cv::Point3f Vision::rotatePointToRobotSide(cv::Point3f p, Vision::RobotSide robotSide)
{
    switch (robotSide)
    {
    case RobotSide::Front:
        // do nothing
        break;

    case RobotSide::Back:
        p.z *= -1;
        p.x *= -1;
        break;

    case RobotSide::Right:
        std::swap(p.x, p.z);
        p.z *= -1;
        break;

    case RobotSide::Left:
        std::swap(p.x, p.z);
        p.x *= -1;

        break;
    }

    return p;
}

void Vision::compute3dPointCloud(cv::Mat &leftCam, cv::Mat &rightCam,
                                 std::vector<cv::Point3f> &pointCloud, std::vector<cv::Vec3b> &colors, Vision::RobotSide robotSide)
{
    cv::Mat disparity;
    computeDisparity(leftCam, rightCam, disparity);

    for (int y = 0; y < disparity.rows; y += 1)
    {
        for (int x = 1; x < disparity.cols - 1; x += 1)
        {
            short disparityMid = disparity.at<short>(y, x);
            short disparityRight = disparity.at<short>(y, x + 1);
            short disparityLeft = disparity.at<short>(y, x - 1);
            
            short usedDisparity = disparityMid;
            if (disparityRight > usedDisparity) usedDisparity = disparityRight;
            if (disparityLeft > usedDisparity) usedDisparity = disparityLeft;

            // only send pixels with disparities greater than 0
            if (usedDisparity > 0)
            {
                cv::Point3f p = convert2dPointTo3d(x, y, usedDisparity);
                if (p.y > 5 || cv::norm(p) > 16 || p.y < -0.5)
                {
                    continue;
                }
                colors.push_back(leftCam.at<cv::Vec3b>(y, x));
                // rotate the point depending on which side these cameras are on
                p = rotatePointToRobotSide(p, robotSide);
                pointCloud.push_back(p);

                if (pointCloud.size() > MAX_POINT_CLOUD_SIZE)
                {
                    break;
                }
            }
        }
    }
    leftCam.release();
    rightCam.release();
}

struct Cluster
{
    std::vector<cv::Point3f> points;
    cv::Point3f center;

    cv::Point3f boundingBoxMin; // holds the max x y and z in the cluster
    cv::Point3f boundingBoxMax; // holds the min x y and z in the cluster

    Cluster(cv::Point3f initPoint)
    {
        points.push_back(initPoint);
        boundingBoxMax = initPoint;
        boundingBoxMin = initPoint;
        center = initPoint;
    }

    // adds a point to the cluster and updates internal fields
    void AddPoint(cv::Point3f p)
    {
        points.push_back(p);
        center = (center * (float)points.size() + p) / (float)(points.size() + 1);

        // update maxes
        boundingBoxMax.x = max(boundingBoxMax.x, p.x);
        boundingBoxMax.y = max(boundingBoxMax.y, p.y);
        boundingBoxMax.z = max(boundingBoxMax.z, p.z);

        // update mins
        boundingBoxMin.x = min(boundingBoxMin.x, p.x);
        boundingBoxMin.y = min(boundingBoxMin.y, p.y);
        boundingBoxMin.z = min(boundingBoxMin.z, p.z);
    }
};

// Compute the similarity score between a given color and yellow, on a scale from 0 to 1
double getSimilarityScore(cv::Vec3b color, cv::Vec3b targetColor)
{
    double dist = 0.0;
    for (int i = 0; i < 3; i++)
    {
        dist += pow(color[i] - targetColor[i], 2);
    }
    dist = sqrt(dist);
    double maxDist = sqrt(pow(255, 2) * 3); // The maximum distance possible between two colors in RGB space
    return 1.0 - (dist / maxDist);                                        // Return the score, normalized to a range of 0 to 1
}

Vision::OpponentCandidate Vision::findOpponent(std::vector<cv::Point3f> &pointCloud, std::vector<cv::Vec3b> &colors)
{
    const double THRESHOLD_Y = 0; // the minimum y position for a point to be considered part of the robot
    const int MIN_POINTS = 200;      // the minimum number of points for a cluster to be considered the robot
    const double MAX_DISTANCE = 2;   // the maximum distance between points in a cluster
    const int MAX_NUM_CLUSTERS = 120;
    const double MAX_DISTANCE_TO_ROBOT = 100.0; // if farther away from us than this, ignore

    // find clusters of points above the y threshold
    std::vector<Cluster> clusters;

    for (int i = 0; i < pointCloud.size(); i++)
    {
        cv::Point3f p = pointCloud[i];

        const cv::Vec3b YELLOW = cv::Vec3b(0, 101, 188);
        double similarityToYellow = getSimilarityScore(colors[i], YELLOW);
        if (similarityToYellow > 0.75)
        {
            // colors[i] = cv::Vec3b(0, 255, 255);
            continue;
        }
        // skip if below threshold
        if (p.y < THRESHOLD_Y)
        {
            continue;
        }

        bool added = false;
        for (int c = 0; c < clusters.size(); c++)
        {
            if (cv::norm(p - clusters[c].center) <= MAX_DISTANCE)
            {
                // colors[i] = cv::Vec3b((c % 10) / 10.0f * 255.0f, ((c + 3) % 10) / 10.0f * 255.0f, ((c + 6) % 10) / 10.0f * 255.0f);
                clusters[c].AddPoint(p);
                added = true;
                break;
            }
        }

        if (!added)
        {
            // add a new cluster
            clusters.push_back(Cluster{p});

            // break if too many clusters
            if (clusters.size() > MAX_NUM_CLUSTERS)
            {
                break;
            }
        }
    }

    // find the largest cluster that has at least MIN_POINTS points
    OpponentCandidate robotCluster;
    for (int c = 0; c < clusters.size(); c++)
    {
        Cluster &cluster = clusters[c];
        double distanceToRobot = cv::norm(cluster.center);

        // skip if not enough points
        if (cluster.points.size() < MIN_POINTS || distanceToRobot > MAX_DISTANCE_TO_ROBOT)
        {
            continue;
        }

        // compute size of bounding box in each dimension
        cv::Point3f boundSizes = cluster.boundingBoxMax - cluster.boundingBoxMin;
        double maxDimension = boundSizes.x;
        maxDimension = max(maxDimension, boundSizes.y);
        maxDimension = max(maxDimension, boundSizes.z);

        // if this was a cube the size of the maximum dimension, how big would it be?
        double volumeIfWasCube = maxDimension * maxDimension * maxDimension;
        double actualVolume = boundSizes.x * boundSizes.y * boundSizes.z;

        const double IDEAL_VOLUME = 10 * 10 * 10;
        // compute score using size
        double thisScore = 0.25 * (abs(actualVolume - IDEAL_VOLUME) / max(actualVolume, IDEAL_VOLUME)) + 0.5 * (1.0 - cv::norm(cluster.center) / 12.0) + 0.25 * cluster.center.y / 0.5;

        // if beats previous best
        if (thisScore > robotCluster.score)
        {
            // set robot cluster to this
            robotCluster = OpponentCandidate{cluster.center, thisScore};
        }
    }

    return robotCluster;
}

// TODO: don't send opponent position
void Vision::runPipeline(cv::Point3f opponentPositionSim, cv::Point3f motionVector)
{
    // compute individual point clouds
    std::vector<cv::Vec3b> colorsFront;
    std::vector<cv::Point3f> pointCloudFront;
    OpponentCandidate opponentCandidateFront;
    compute3dPointCloud(cameraTL.getFrame(), cameraTR.getFrame(), pointCloudFront, colorsFront, Vision::RobotSide::Front);
    // opponentCandidateFront = findOpponent(pointCloudFront, colorsFront);

    std::vector<cv::Vec3b> colorsBack;
    std::vector<cv::Point3f> pointCloudBack;
    OpponentCandidate opponentCandidateBack;
    compute3dPointCloud(cameraBL.getFrame(), cameraBR.getFrame(), pointCloudBack, colorsBack, Vision::RobotSide::Back);
    // opponentCandidateBack = findOpponent(pointCloudBack, colorsBack);

    std::vector<cv::Vec3b> colorsLeft;
    std::vector<cv::Point3f> pointCloudLeft;
    OpponentCandidate opponentCandidateLeft;
    compute3dPointCloud(cameraLL.getFrame(), cameraLR.getFrame(), pointCloudLeft, colorsLeft, Vision::RobotSide::Left);
    // opponentCandidateLeft = findOpponent(pointCloudLeft, colorsLeft);

    std::vector<cv::Vec3b> colorsRight;
    std::vector<cv::Point3f> pointCloudRight;
    OpponentCandidate opponentCandidateRight;
    compute3dPointCloud(cameraRL.getFrame(), cameraRR.getFrame(), pointCloudRight, colorsRight, Vision::RobotSide::Right);
    // opponentCandidateRight = findOpponent(pointCloudRight, colorsRight);

    // combine point clouds into one
    std::vector<cv::Vec3b> colors;
    std::vector<cv::Point3f> pointCloud;

    pointCloud.insert(pointCloud.end(), pointCloudFront.begin(), pointCloudFront.end());
    colors.insert(colors.end(), colorsFront.begin(), colorsFront.end());

    pointCloud.insert(pointCloud.end(), pointCloudBack.begin(), pointCloudBack.end());
    colors.insert(colors.end(), colorsBack.begin(), colorsBack.end());

    pointCloud.insert(pointCloud.end(), pointCloudLeft.begin(), pointCloudLeft.end());
    colors.insert(colors.end(), colorsLeft.begin(), colorsLeft.end());

    pointCloud.insert(pointCloud.end(), pointCloudRight.begin(), pointCloudRight.end());
    colors.insert(colors.end(), colorsRight.begin(), colorsRight.end());

    // find best opponent position
    OpponentCandidate opponent = {cv::Point3f(0, 0, 0), 0};

    if (opponentCandidateFront.score > opponent.score)
    {
        opponent = opponentCandidateFront;
    }
    if (opponentCandidateBack.score > opponent.score)
    {
        opponent = opponentCandidateBack;
    }
    if (opponentCandidateLeft.score > opponent.score)
    {
        opponent = opponentCandidateLeft;
    }
    if (opponentCandidateRight.score > opponent.score)
    {
        opponent = opponentCandidateRight;
    }

    // if gameloop created, submit vertices + opponent position
    if (pGameLoop)
    {
        pGameLoop->SetPointCloudVerts(pointCloud, colors);
        pGameLoop->SetOpponentPosition({opponentPositionSim.x, opponentPositionSim.y, opponentPositionSim.z});//{opponent.pos.x, 0, opponent.pos.z}
        std::vector<GameLoop::Line> lines = {};
        lines.push_back(GameLoop::Line{cv::Point3f(0, 0, 0),
                                       opponentPositionSim, //{opponent.pos.x, 0, opponent.pos.z}
                                       cv::Vec3b(0, 255, 255)});

        lines.push_back(GameLoop::Line{cv::Point3f(0, 0, 0),
                                       motionVector,
                                       cv::Vec3b(255, 0, 0)});
        pGameLoop->SetPathPlanningLines(lines);
    }
}
