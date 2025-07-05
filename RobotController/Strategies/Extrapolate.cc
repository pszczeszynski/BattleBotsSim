#include "Extrapolate.h"

OdometryData ExtrapolateOpponentPos(double seconds, double max_prediction_time)
{
    // extrapolate the opponent's position into the future
    OdometryData opponentData = RobotController::GetInstance().odometry.Opponent();
    OdometryData ourData = RobotController::GetInstance().odometry.Robot();

    double distanceToOpponent = cv::norm(ourData.robotPosition - opponentData.robotPosition);

    double extrapolationTime = std::min((distanceToOpponent / 100.0 * seconds), max_prediction_time);
    extrapolationTime += Clock::programClock.getElapsedTime();

    opponentData._Extrapolate(extrapolationTime);

    // clip within field bounds
    opponentData.robotPosition.x = std::max(0.0f, std::min(opponentData.robotPosition.x, (float) WIDTH));
    opponentData.robotPosition.y = std::max(0.0f, std::min(opponentData.robotPosition.y, (float) HEIGHT));


    return opponentData;
}
