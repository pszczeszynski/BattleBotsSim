#include "MovementStrategy.h"
#include "RobotController.h"
#include "UIWidgets/ImageWidget.h"

// ctor
MovementStrategy::MovementStrategy()
{

}

/*
Requirements:

1. Don't touch the opponent
2. Don't touch walls
3. Preserve momentum as much as possible
4. iteratively build solution => step by step
5. if at desireable distance away, pace and mess with them to promote attack
*/

/**
 * Plans where the robot should go to avoid the opponent
*/
cv::Point2f MovementStrategy::AvoidStrategy()
{
    const int STEP_SIZE = 10;
    static ImageWidget widget{"Strategy Graphic", false};
    static cv::Mat strategyGraphic{WIDTH / STEP_SIZE, HEIGHT / STEP_SIZE, CV_8UC3, cv::Scalar(0, 0, 0)};

    cv::Point2f ret = cv::Point2f(0, 0);

    // 1. get as far away from opponent as possible (within limits)

    // get drawing image from rc
    cv::Mat& drawingImage = RobotController::GetInstance().GetDrawingImage();

    // clear strategy graphic
    cv::Mat strategyGraphicSmall{WIDTH / STEP_SIZE, HEIGHT / STEP_SIZE, CV_8UC3, cv::Scalar(0, 0, 0)};

    // get our position
    cv::Point2f ourPos = RobotOdometry::Robot().GetPosition() / STEP_SIZE;
    // get opponent position
    cv::Point2f opponentPos = RobotOdometry::Opponent().GetPosition() / STEP_SIZE;

    // go through every point in the drawing image with step size of 10
    for (int i = 0; i < strategyGraphicSmall.size().height; i += 1)
    {
        for (int j = 0; j < strategyGraphicSmall.size().width; j += 1)
        {
            // draw a circle at the point. More red means closer to opponent

            // get the point
            cv::Point2f point = cv::Point2f(j, i);

            // get the distance to the opponent
            float distToOpponent = cv::norm(point - opponentPos);

            // 1. get as far away from opponent as possible (within limits)
            double opponentProximityPenalty = 1.0 - distToOpponent / (strategyGraphicSmall.size().width * 0.5);
            opponentProximityPenalty = max(0.0, opponentProximityPenalty);
            opponentProximityPenalty *= opponentProximityPenalty;

            // 2. don't touch walls
            // compute dist to nearest edge
            int distToEdge = min(min(point.x, point.y), min(strategyGraphicSmall.size().width - point.x,
                                                            strategyGraphicSmall.size().height - point.y));
            // compute penalty
            double wallProximityPenalty = 1.0 - distToEdge / (strategyGraphicSmall.size().width * 0.5);

            // square
            wallProximityPenalty *= wallProximityPenalty;

            // put pixel
            strategyGraphicSmall.at<cv::Vec3b>(i, j) = cv::Vec3b(255 * wallProximityPenalty, 0, 255 * opponentProximityPenalty);
        }
    }
    
    // resize up
    cv::resize(strategyGraphicSmall, strategyGraphic, cv::Size(WIDTH, HEIGHT));

    // add weighted with drawing image
    cv::addWeighted(drawingImage, 0.2, strategyGraphic, 0.8, 0, strategyGraphic);

    widget.UpdateMat(strategyGraphic);


    // plot the point to go to
    cv::circle(drawingImage, ret, 5, cv::Scalar(0, 255, 0), 2);

    // return the point to go to
    return ret;
}