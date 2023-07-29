#include "RobotTracker.h"
#include "MathUtils.h"
#include "OpponentProfile.h"

RobotTracker::RobotTracker(cv::Point2f initialPosition) :
    position{initialPosition},
    isValid{false}
{
}

/**
 * @brief returns the robot tracker for our robot
*/
RobotTracker& RobotTracker::Robot()
{
    static RobotTracker robot(cv::Point2f(0,0));
    return robot;
}

/**
 * @brief returns the robot tracker for the opponent
*/
RobotTracker& RobotTracker::Opponent()
{
    static RobotTracker opponent(cv::Point2f(0,0));
    return opponent;
}

// the displacement required to update the angle
#define DIST_BETWEEN_ANG_UPDATES_PX 5

// 0 = no change, 1 = full change
#define MOVING_AVERAGE_RATE 1.0

// how fast the robot needs to be moving to update the angle
#define VELOCITY_THRESH_FOR_ANGLE_UPDATE 50

/**
 * @brief updates the angle of the robot using the velocity
*/
Angle RobotTracker::CalcAnglePathTangent()
{
    Angle retAngleRad;

    // if we have travelled a certain distance since the last update
    // and we are moving fast enough
    if (norm(position - lastPositionWhenUpdatedAngle) >= DIST_BETWEEN_ANG_UPDATES_PX &&
        norm(velocity) > VELOCITY_THRESH_FOR_ANGLE_UPDATE)
    {
        // calcualte the change from the last update
        cv::Point2f delta = position - lastPositionWhenUpdatedAngle;
        // update the angle but only by half the change
        retAngleRad = (Angle(atan2(delta.y, delta.x)) - retAngleRad) * MOVING_AVERAGE_RATE + retAngleRad;
        // save the last position
        lastPositionWhenUpdatedAngle = position;
        // set the flag to true
        visualAngleValid = true;
    }
    else
    {
        visualAngleValid = false;
    }

    return retAngleRad;
}

/**
 * @brief updates with both visual and imu information. this should only be called for our robot (since we have our imu)
 * However, sometimes we don't have the visual information in which we call UpdateIMUOnly
*/
#define VISUAL_INFO_WEIGHT 0.1
#define NEW_VISUAL_VELOCITY_WEIGHT 1.0

void RobotTracker::UpdateVisionAndIMU(MotionBlob& blob, cv::Mat& frame, RobotIMUData& imuData)
{
    // predict the new position using just the velocity
    cv::Point2f averageVelocityLastUpdate = (velocity + imuData.velocity) / 2;
    // predict the new position
    cv::Point2f predictedPosition = position + averageVelocityLastUpdate * lastUpdate.getElapsedTime();
    // use the blob's center for the visual position
    cv::Point2f visualPosition = blob.center;
    // do a weighted average of the predicted position and the visual position to smooth out the noise in the visual position
    cv::Point2f weightedAveragePosition = predictedPosition * (1 - VISUAL_INFO_WEIGHT) + visualPosition * VISUAL_INFO_WEIGHT;


    // visual velocity
    // derive velocity using the visual information
    cv::Point2f visualVelocity = (blob.center - position) / lastUpdate.getElapsedTime();
    cv::Point2f weightedAverageVelocity = visualVelocity * NEW_VISUAL_VELOCITY_WEIGHT + velocity * (1 - NEW_VISUAL_VELOCITY_WEIGHT);
    // update using the weighted average
    UpdateSetPosAndVel(weightedAveragePosition, weightedAverageVelocity);





    // set the angle to the imu angle
    double angleImu = angle + (Angle(imuData.angle) - Angle(lastIMUAngle));
    double angleVisual = CalcAnglePathTangent();
    if (visualAngleValid)
    {
        // if we have visual information, use the visual angle
        angle = Angle(angleImu * (1.0 - VISUAL_INFO_WEIGHT) + angleVisual * VISUAL_INFO_WEIGHT);
    }
    else
    {
        angle = Angle(angleImu);
    }

    // save the last angle
    lastIMUAngle = angleImu;
}


/**
 * @brief updates with just visual information. this should only be called for the opponent (since we don't have their imu)
 * Updates the position of the robot to the given position.
 * @param blob - the MotionBlob to update to
*/
void RobotTracker::UpdateVisionOnly(MotionBlob& blob, cv::Mat& frame)
{
    // derive velocity using the visual information
    cv::Point2f visualVelocity = (blob.center - position) / lastUpdate.getElapsedTime();

    cv::Point2f weightedAverageVelocity = visualVelocity * NEW_VISUAL_VELOCITY_WEIGHT + velocity * (1 - NEW_VISUAL_VELOCITY_WEIGHT);
    // use the blob's center for the visual position
    cv::Point2f visualPosition = blob.center;
    // update using the visual information
    UpdateSetPosAndVel(visualPosition, weightedAverageVelocity);

    // update our angle using the velocity
    angle = CalcAnglePathTangent();
}

/**
 * Called when we only have the imu information. This should only be called for our robot (since we have our imu)
*/
void RobotTracker::UpdateIMUOnly(RobotIMUData& imuData)
{
    // calcualte the average velocity of the last update's frame
    cv::Point2f averageVelocityLastUpdate = (velocity + imuData.velocity) / 2;
    // predict the new position
    cv::Point2f newPosition = position + averageVelocityLastUpdate * lastUpdate.getElapsedTime();
    // update normally
    UpdateSetPosAndVel(newPosition, imuData.velocity);
    // set our angle to the imu angle
    angle = Angle(imuData.angle);
}

void RobotTracker::UpdateSetPosAndVel(cv::Point2f position, cv::Point2f velocity)
{
    // update our velocity and position
    this->velocity = velocity;
    this->position = position;

    isValid = true;

    // mark this as the last update time
    lastUpdate.markStart();
}

/**
 * @brief getPosition
 * @return the current position of the robot.
*/
cv::Point2f RobotTracker::getPosition()
{
    return position;
}

Angle RobotTracker::getAngle()
{
    return angle;
}

void RobotTracker::invalidate()
{
    velocity = cv::Point2f(0,0);
    isValid = false;
}

cv::Point2f RobotTracker::GetVelocity()
{
    return velocity;
}