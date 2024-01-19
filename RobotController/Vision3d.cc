#include "Vision3d.h"

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

#define OVERHEAD_WIDTH 1280
#define OVERHEAD_HEIGHT 1280

Vision3d::Vision3d(ICameraReceiver &overheadCamL, ICameraReceiver &overheadCamR)
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
    robotTrackers.push_back(RobotOdometry(cv::Point2f(0,0)));
    robotTrackers.push_back(RobotOdometry(cv::Point2f(10000, 10000)));

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

/**
 * Converts a point cloud to an overhead image
*/
void Vision3d::convertPointCloudToOverhead(std::vector<cv::Point3f> pointCloud, std::vector<cv::Vec3b> colors, cv::Mat& dstOverhead)
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
}

/**
 * This is the 3d version of the pipeline
*/
void Vision3d::runPipeline()
{
    overheadCamL.GetFrame(currFrameL);
    overheadCamR.GetFrame(currFrameR);

    std::cout << "currFrameL.size():  " << currFrameL.size() << std::endl;

    alignImages(currFrameL, currFrameR, currFrameL, currFrameR);

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




void equalizeHistogram3Channels(cv::Mat& src, cv::Mat& dst)
{
    // this version just normalizes each channel
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    for (int i = 0; i < 3; i++)
    {
        cv::normalize(channels[i], channels[i], 0, 255, cv::NORM_MINMAX);
    }
    cv::merge(channels, dst);
}

void Vision3d::correctStereoColor(cv::Mat &leftImage, cv::Mat &rightImage, cv::Mat &leftImageCorrected, cv::Mat &rightImageCorrected)
{
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

void Vision3d::alignImages(const cv::Mat& src1, const cv::Mat& src2, cv::Mat& dst1, cv::Mat& dst2)
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

const cv::Mat& Vision3d::GetBirdsEyeImageL()
{
    return previousFrameL;
}

const cv::Mat& Vision3d::GetBirdsEyeImageR()
{
    return previousFrameR;
}

#define CAMERA_ANGLE_TOWARDS_GROUND_RAD 40.767 * TO_RAD
cv::Point3f Vision3d::convert2dPointTo3d(int x, int y, short disparity)
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

void Vision3d::computeDisparity(const cv::Mat &left, const cv::Mat &right, cv::Mat &disparity, cv::Mat &disparityNormalized)
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
}


#define POINT_CLOUD_MIN_Y -1.155
#define POINT_CLOUD_MAX_Y -0.5
void Vision::compute3dPointCloud(cv::Mat &leftCam, cv::Mat &rightCam,
                                 std::vector<cv::Point3f> &pointCloud, std::vector<cv::Vec3b> &colors)
{
    cv::Mat disparity;
    cv::Mat disparityNormalized;
    computeDisparity(leftCam, rightCam, disparity, disparityNormalized);

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

            if (p.y < POINT_CLOUD_MIN_Y || p.y > POINT_CLOUD_MAX_Y)
            {
                continue;
            }

            colors.push_back(color);
            pointCloud.push_back(p);
        }
    }
}
