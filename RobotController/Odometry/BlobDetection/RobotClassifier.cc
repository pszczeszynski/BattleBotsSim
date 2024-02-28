#include "RobotClassifier.h"
#include "../../Globals.h"
#include "../../RobotConfig.h"

RobotClassifier::RobotClassifier(void)
{
    robotCalibrationData.meanColor = cv::Scalar(0, 0, 100);
    robotCalibrationData.diameter = 50;

    opponentCalibrationData.meanColor = cv::Scalar(100, 100, 100);
    opponentCalibrationData.diameter = 50;
}

void visualizeHistogram(const cv::Mat &hist)
{
    int hist_w = 512, hist_h = 400;
    int bin_w = cvRound((double)hist_w / hist.rows);
    cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

    /// Normalize the result to [ 0, histImage.rows ]
    cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());

    for (int i = 1; i < hist.rows; i++)
    {
        cv::line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
                 cv::Point(bin_w * (i), hist_h - cvRound(hist.at<float>(i))),
                 cv::Scalar(255, 0, 0), 2, 8, 0);
    }

    /// Display
    cv::namedWindow("calcHist Demo", cv::WINDOW_AUTOSIZE);
    cv::imshow("calcHist Demo", histImage);
    cv::waitKey(1);
}

cv::Mat calcHistogram(const cv::Mat &img, const cv::Mat &mask)
{
    // The image may be B&W or color. If black and white, use the single channel for all
    std::vector<cv::Mat> bgr_planes;
    cv::split(img, bgr_planes);

    int histSize = 256;
    float range[] = {0, 256}; // the upper boundary is exclusive
    const float *histRange = {range};

    cv::Mat b_hist, g_hist, r_hist;

    cv::calcHist(&bgr_planes[0], 1, 0, mask, b_hist, 1, &histSize, &histRange);

    if (bgr_planes.size() >= 2)
    {
        cv::calcHist(&bgr_planes[1], 1, 0, mask, g_hist, 1, &histSize, &histRange);
    }
    else
    {
        g_hist = b_hist;
    }

    if (bgr_planes.size() >= 3)
    {
        cv::calcHist(&bgr_planes[2], 1, 0, mask, r_hist, 1, &histSize, &histRange);
    }
    else
    {
        r_hist = b_hist;
    }

    cv::Mat hist;
    cv::vconcat(std::vector<cv::Mat>{b_hist, g_hist, r_hist}, hist);

    return hist;
}

// returns a number between 0 and 1, where 0 means the histograms are identical
double compareHistograms(const cv::Mat &hist1, const cv::Mat &hist2)
{
    // check if valid to compare
    if (hist1.rows != hist2.rows || hist1.cols != hist2.cols)
    {
        return 0;
    }

    return cv::compareHist(hist1, hist2, cv::HISTCMP_BHATTACHARYYA);
}

cv::Scalar RobotClassifier::GetMeanColorOfBlob(MotionBlob &blob, cv::Mat &frame, cv::Mat &motionImage)
{
    cv::Rect reduced = blob.rect;
    // reduced.x += reduced.width / 4;
    // reduced.y += reduced.height / 4;
    // reduced.width /= 2;
    // reduced.height /= 2;

    cv::Mat croppedFrame = frame(reduced);
    cv::Mat croppedMotion = motionImage(reduced);
    cv::Scalar mean = cv::mean(croppedFrame, croppedMotion);
    return mean;
}

void RobotClassifier::RecalibrateRobot(RobotCalibrationData &data, MotionBlob &blob, cv::Mat &frame, cv::Mat &motionImage)
{
    data.meanColor = GetMeanColorOfBlob(blob, frame, motionImage);
    data.diameter = (blob.rect.width + blob.rect.height) / 2;
    data.histogram = calcHistogram(frame(blob.rect), motionImage(blob.rect));

    // fill DRAWING_IMAGE with red at (blob.rect masked with motionimage)
    cv::Mat redImage = cv::Mat::zeros(frame.size(), CV_8UC3);
    redImage.setTo(cv::Scalar(0, 0, 255), motionImage);
    SAFE_DRAW
    cv::addWeighted(drawingImage, 1, redImage, 0.5, 0, drawingImage);
    END_SAFE_DRAW

    // visualizeHistogram(data.histogram);

    // if (&data == &robotCalibrationData)
    // {
    //     std::cout << "recalibrated robot to color: " << data.meanColor << std::endl;
    // }
    // else
    // {
    //     std::cout << "recalibrated opponent to color: " << data.meanColor << std::endl;
    // }
}

#define HISTOGRAM_WEIGHT 0.0
#define DISTNACE_WEIGHT 1.0
/**
 * Returns a number that is more negative the more we think the robot is us.
 *
 * What does that look for?
 * Being more blue in the lastFrame
 */
double RobotClassifier::ClassifyBlob(MotionBlob &blob, cv::Mat &frame, cv::Mat &motionImage, OdometryData &robotData, OdometryData &opponentData)
{
    // now let's take their location into account
    double distanceToRobot = cv::norm(robotData.robotPosition - blob.center);
    double distanceToOpponent = cv::norm(opponentData.robotPosition - blob.center);

    // normalize the distance score
    double distanceScoreNormalized = (distanceToRobot - distanceToOpponent) / (distanceToRobot + distanceToOpponent);

    // weight and sum the scores
    return distanceScoreNormalized;
}

/**
 * Returns the closest blob to the given point and removes it from the list
 */
MotionBlob GetClosestBlobAndRemove(std::vector<MotionBlob> &blobs, cv::Point2f point)
{
    int index = 0;
    double minDist = 10000000;

    for (int i = 0; i < blobs.size(); i++)
    {
        double dist = cv::norm(blobs[i].center - point);
        if (dist < minDist)
        {
            minDist = dist;
            index = i;
        }
    }

    MotionBlob closest = blobs[index];
    blobs.erase(blobs.begin() + index);
    return closest;
}

#define MATCHING_DIST_THRESHOLD 100 * WIDTH / 720
#define TIME_UNTIL_BIGGER_THRESH_SECONDS 3
/**
 * @brief Classifies the blobs and returns the robot and opponent blobs
 * @param blobs the blobs to classify
 * @param frame the latest frame from the camera
 * @param motionImage a frame with only the motion as white pixels
 */
VisionClassification RobotClassifier::ClassifyBlobs(std::vector<MotionBlob> &blobs, cv::Mat &frame, cv::Mat &motionImage, OdometryData &robotData, OdometryData &opponentData)
{
    VisionClassification classificationResult;

    static int lastBlobsSize = 0;
    static Clock noRobotClock;
    static Clock noOpponentClock;

    double matchingDistThresholdRobot = MATCHING_DIST_THRESHOLD;
    double matchingDistThresholdOpponent = MATCHING_DIST_THRESHOLD;

    // // if haven't matched the robot for a while, increase the dist threshold for it
    // if (noRobotClock.getElapsedTime() > 3)
    // {
    //     matchingDistThresholdRobot = WIDTH * 3;
    // }

    // // if we haven't matched the opponent in a while, increase the dist threshold for it
    // if (noOpponentClock.getElapsedTime() > 3)
    // {
    //     matchingDistThresholdOpponent = WIDTH * 3;
    // }

    // if only one blob
    if (blobs.size() == 1)
    {
        // Note: RobotOdometry gets updates position from other sources, thus GetPosition can change
        // while we are running this code. Is it ok to process this frame's blob detection with a position
        // that may be based on a newer frame we haven't processed yet? This is probably ok since this code
        // is one of the fastest codes run, and if better information on GetPosition arrives from other algorithms
        // then we want to search for blobs near it.
        double distanceToRobot = cv::norm(robotData.robotPosition - blobs[0].center);
        double distanceToOpponent = cv::norm(opponentData.robotPosition - blobs[0].center);

        bool isRobot = ClassifyBlob(blobs[0], frame, motionImage, robotData, opponentData) <= 0;

        // if the blob is really close to both robots
        if (distanceToRobot < 10 && distanceToOpponent < 10)
        {
            // choose the one with more velocity last time
            isRobot = cv::norm(robotData.robotVelocity) > cv::norm(opponentData.robotVelocity);
        }

        // if this is the robot (not the opponent)
        if (isRobot)
        {
            if (distanceToRobot < matchingDistThresholdRobot &&
                sqrt(blobs[0].rect.area()) >= MIN_ROBOT_BLOB_SIZE &&
                sqrt(blobs[0].rect.area()) <= MAX_ROBOT_BLOB_SIZE)
            {
                RecalibrateRobot(robotCalibrationData, blobs[0], frame, motionImage);
                classificationResult.SetRobot(blobs[0]);
                noRobotClock.markStart();
            }
        }
        else
        {
            // make sure it's close enough and the size is big enough
            if (distanceToOpponent < matchingDistThresholdOpponent &&
                sqrt(blobs[0].rect.area()) >= MIN_OPPONENT_BLOB_SIZE &&
                sqrt(blobs[0].rect.area()) <= MAX_OPPONENT_BLOB_SIZE)
            {

                RecalibrateRobot(opponentCalibrationData, blobs[0], frame, motionImage);
                classificationResult.SetOpponent(blobs[0]);
                noRobotClock.markStart();
            }
        }
    }
    // otherwise have more than 2 blobs, only use the first 2 blobs
    else if (blobs.size() >= 2)
    {
        std::vector<MotionBlob> blobsToChooseFrom = {};
        std::vector<MotionBlob> filtered = {};
        // copy over all the blobs
        std::copy(blobs.begin(), blobs.end(), std::back_inserter(blobsToChooseFrom));
        filtered.push_back(GetClosestBlobAndRemove(blobsToChooseFrom, robotData.robotPosition));
        filtered.push_back(GetClosestBlobAndRemove(blobsToChooseFrom, opponentData.robotPosition));

        double firstIsRobot = ClassifyBlob(filtered[0], frame, motionImage, robotData, opponentData);
        double secondIsRobot = ClassifyBlob(filtered[1], frame, motionImage, robotData, opponentData);
        double preference = firstIsRobot - secondIsRobot;

        MotionBlob &robot = preference <= 0 ? filtered[0] : filtered[1];
        MotionBlob &opponent = preference <= 0 ? filtered[1] : filtered[0];

        RecalibrateRobot(robotCalibrationData, robot, frame, motionImage);
        RecalibrateRobot(opponentCalibrationData, opponent, frame, motionImage);

        // if the robot blob is close to the robot tracker
        if (norm(robot.center - robotData.robotPosition) < matchingDistThresholdRobot &&
            sqrt(robot.rect.area()) >= MIN_ROBOT_BLOB_SIZE &&
            sqrt(robot.rect.area()) <= MAX_ROBOT_BLOB_SIZE)
        {
            classificationResult.SetRobot(robot);
            noRobotClock.markStart();
        }

        // if the opponent blob is close to the opponent tracker
        if (norm(opponent.center - opponentData.robotPosition) < matchingDistThresholdOpponent &&
            sqrt(opponent.rect.area()) >= MIN_OPPONENT_BLOB_SIZE &&
            sqrt(opponent.rect.area()) <= MAX_OPPONENT_BLOB_SIZE)
        {
            classificationResult.SetOpponent(opponent);
            noRobotClock.markStart();
        }
    }

    lastBlobsSize = blobs.size();

    // return our classification
    return classificationResult;
}
