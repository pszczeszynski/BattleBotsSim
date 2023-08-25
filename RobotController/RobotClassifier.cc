#include "RobotClassifier.h"
#include "Globals.h"
#include "Input/Input.h"
#include "RobotConfig.h"

RobotClassifier* RobotClassifier::instance = nullptr;

RobotClassifier::RobotClassifier()
{
    robotCalibrationData.meanColor = cv::Scalar(0, 0, 100);
    robotCalibrationData.diameter = 50;

    opponentCalibrationData.meanColor = cv::Scalar(100, 100, 100);
    opponentCalibrationData.diameter = 50;

    RobotClassifier::instance = this;
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
    std::vector<cv::Mat> bgr_planes;
    cv::split(img, bgr_planes);


    int histSize = 256;
    float range[] = {0, 256}; // the upper boundary is exclusive
    const float *histRange = {range};

    cv::Mat b_hist, g_hist, r_hist;

    cv::calcHist(&bgr_planes[0], 1, 0, mask, b_hist, 1, &histSize, &histRange);
    cv::calcHist(&bgr_planes[1], 1, 0, mask, g_hist, 1, &histSize, &histRange);
    cv::calcHist(&bgr_planes[2], 1, 0, mask, r_hist, 1, &histSize, &histRange);

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

void RobotClassifier::SwitchRobots()
{
    cv::Point2f savedPos = RobotOdometry::Robot().GetPosition();
    cv::Point2f savedVel = RobotOdometry::Robot().GetVelocity();
    // swap the positions and velocities of the robots
    RobotOdometry::Robot().UpdateForceSetPosAndVel(RobotOdometry::Opponent().GetPosition(), RobotOdometry::Opponent().GetVelocity());
    RobotOdometry::Opponent().UpdateForceSetPosAndVel(savedPos, savedVel);
}

cv::Scalar RobotClassifier::GetMeanColorOfBlob(MotionBlob& blob, cv::Mat& frame, cv::Mat& motionImage)
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

void RobotClassifier::RecalibrateRobot(RobotCalibrationData& data, MotionBlob& blob, cv::Mat& frame, cv::Mat& motionImage)
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
double RobotClassifier::ClassifyBlob(MotionBlob& blob, cv::Mat& frame, cv::Mat& motionImage)
{
    // now let's take their location into account
    double distanceToRobot = cv::norm(RobotOdometry::Robot().GetPosition() - blob.center);
    double distanceToOpponent = cv::norm(RobotOdometry::Opponent().GetPosition() - blob.center);

    // normalize the distance score
    double distanceScoreNormalized = (distanceToRobot - distanceToOpponent) / (distanceToRobot + distanceToOpponent);

    // weight and sum the scores
    return distanceScoreNormalized;
}

#define MATCHING_DIST_THRESHOLD 100 * WIDTH / 720
#define TIME_UNTIL_BIGGER_THRESH_SECONDS 3
/**
 * @brief Classifies the blobs and returns the robot and opponent blobs
 * @param blobs the blobs to classify
 * @param frame the latest frame from the camera
 * @param motionImage a frame with only the motion as white pixels
*/
VisionClassification RobotClassifier::ClassifyBlobs(std::vector<MotionBlob>& blobs, cv::Mat& frame, cv::Mat& motionImage)
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
        double distanceToRobot = cv::norm(RobotOdometry::Robot().GetPosition() - blobs[0].center);
        double distanceToOpponent = cv::norm(RobotOdometry::Opponent().GetPosition() - blobs[0].center);

        bool isRobot = ClassifyBlob(blobs[0], frame, motionImage) <= 0;

        // if the blob is really close to both robots
        if (distanceToRobot < 10 && distanceToOpponent < 10)
        {
            // choose the one with more velocity last time
            isRobot = cv::norm(RobotOdometry::Robot().GetVelocity()) > cv::norm(RobotOdometry::Opponent().GetVelocity());
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
                sqrt(blobs[0].rect.area()) <= MIN_OPPONENT_BLOB_SIZE)
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
        double firstIsRobot = ClassifyBlob(blobs[0], frame, motionImage);
        double secondIsRobot = ClassifyBlob(blobs[1], frame, motionImage);
        double preference = firstIsRobot - secondIsRobot;

        MotionBlob& robot = preference <= 0 ? blobs[0] : blobs[1];
        MotionBlob& opponent = preference <= 0 ? blobs[1] : blobs[0];

        RecalibrateRobot(robotCalibrationData, robot, frame, motionImage);
        RecalibrateRobot(opponentCalibrationData, opponent, frame, motionImage);

        // if the robot blob is close to the robot tracker
        if (norm(robot.center - RobotOdometry::Robot().GetPosition()) < matchingDistThresholdRobot &&
            sqrt(robot.rect.area()) >= MIN_ROBOT_BLOB_SIZE &&
            sqrt(robot.rect.area()) <= MAX_ROBOT_BLOB_SIZE)
        {
            classificationResult.SetRobot(robot);
            noRobotClock.markStart();
        }

        // if the opponent blob is close to the opponent tracker
        if (norm(opponent.center - RobotOdometry::Opponent().GetPosition()) < matchingDistThresholdOpponent &&
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
