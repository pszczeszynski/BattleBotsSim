#include "RobotClassifier.h"
#include "../../Globals.h"
#include "../../RobotConfig.h"
#include "../../RobotController.h"

RobotClassifier::RobotClassifier(void)
{
    robotCalibrationData.meanColor = cv::Scalar(0, 0, 100);
    robotCalibrationData.diameter = 50;

    opponentCalibrationData.meanColor = cv::Scalar(100, 100, 100);
    opponentCalibrationData.diameter = 50;
}

#define DISTNACE_WEIGHT 1.0

/**
 * Returns a number that is more negative the more we think the robot is us.
 *
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
    static int lastNeuralID = -1;
    static Clock noRobotClock;
    static Clock noOpponentClock;

    double matchingDistThresholdRobot = MATCHING_DIST_THRESHOLD;
    double matchingDistThresholdOpponent = MATCHING_DIST_THRESHOLD;

    CVPosition& cvPosition = RobotController::GetInstance().odometry.GetNeuralOdometry();

    // check if the neural network has a new position
    if (cvPosition.IsRunning() && cvPosition.NewDataValid(lastNeuralID))
    {
        lastNeuralID = cvPosition.GetData().id;

        // get the latest position from the neural network
        cv::Point2f neuralPosition = cvPosition.GetData().robotPosition;

        if (blobs.size() > 0)
        {
            // find closest blob to the neural position
            MotionBlob closest = GetClosestBlobAndRemove(blobs, neuralPosition);

            // if the closest blob is close enough to the neural position
            if (cv::norm(closest.center - neuralPosition) < matchingDistThresholdRobot)
            {
                // set robot
                classificationResult.SetRobot(closest);
                noRobotClock.markStart();
            }
        }

        // remove any blob within the matching threshold of the neural position
        for (int i = 0; i < blobs.size(); i++)
        {
            if (cv::norm(blobs[i].center - neuralPosition) < matchingDistThresholdRobot)
            {
                blobs.erase(blobs.begin() + i);
                i--;
            }
        }

        if (blobs.size() > 0)
        {
            // find the closest blob to the opponent
            MotionBlob closest = GetClosestBlobAndRemove(blobs, opponentData.robotPosition);

            // if close to the opponent
            if (cv::norm(closest.center - opponentData.robotPosition) < matchingDistThresholdOpponent && 
                // and far from us
                cv::norm(closest.center - robotData.robotPosition) > matchingDistThresholdRobot)
            {
                // set opponent
                classificationResult.SetOpponent(closest);
                noOpponentClock.markStart(); 
            }
        }

        return classificationResult;
    }

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
