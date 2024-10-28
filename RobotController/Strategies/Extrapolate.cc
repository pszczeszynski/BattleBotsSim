#include "Extrapolate.h"

OdometryData ExtrapolateOpponentPos()
{
    // extrapolate the opponent's position into the future
    OdometryData opponentData = RobotController::GetInstance().odometry.Opponent();
    OdometryData ourData = RobotController::GetInstance().odometry.Robot();

    double distanceToOpponent = cv::norm(ourData.robotPosition - opponentData.robotPosition);

    double extrapolationTime = std::min(distanceToOpponent / 100.0 * (OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0), MAX_PREDICTION_TIME);
    extrapolationTime += Clock::programClock.getElapsedTime();

    opponentData.Extrapolate(extrapolationTime);

    // clip within field bounds
    opponentData.robotPosition.x = std::max(0.0f, std::min(opponentData.robotPosition.x, (float) WIDTH));
    opponentData.robotPosition.y = std::max(0.0f, std::min(opponentData.robotPosition.y, (float) HEIGHT));


    return opponentData;
}