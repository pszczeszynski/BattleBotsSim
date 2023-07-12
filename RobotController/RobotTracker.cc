#include "RobotTracker.h"
#include "MathUtils.h"
#include "OpponentProfile.h"

RobotTracker::RobotTracker(cv::Point2f initialPosition) :
    position{initialPosition},
    isValid{false}
{
}

// the displacement required to update the angle
#define DIST_BETWEEN_ANG_UPDATES_PX 5

// 0 = no change, 1 = full change
#define MOVING_AVERAGE_RATE 0.5

// how fast the robot needs to be moving to update the angle
#define VELOCITY_THRESH_FOR_ANGLE_UPDATE 50

/**
 * @brief updates the angle of the robot using the velocity
*/
void RobotTracker::UpdateAngle()
{
    static Angle lastAngleRaw(0);

    // if we have travelled a certain distance since the last update
    // and we are moving fast enough
    if (norm(position - lastPositionWhenUpdatedAngle) >= DIST_BETWEEN_ANG_UPDATES_PX &&
        norm(velocity) > VELOCITY_THRESH_FOR_ANGLE_UPDATE)
    {
        // calcualte the change from the last update
        cv::Point2f delta = position - lastPositionWhenUpdatedAngle;
        // update the angle but only by half the change
        angle = (Angle(atan2(delta.y, delta.x)) - angle) * MOVING_AVERAGE_RATE + angle;
        // save the last position
        lastPositionWhenUpdatedAngle = position;
    }
}

/**
 * @brief updates with both visual and imu information. this should only be called for our robot (since we have our imu)
 * However, sometimes we don't have the visual information in which we call UpdateIMUOnly
*/
#define VISUAL_INFO_WEIGHT 0.1
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
    // update using the weighted average
    update(weightedAveragePosition, imuData.velocity);
}


#define NEW_VELOCITY_WEIGHT 0.1
/**
 * @brief updates with just visual information. this should only be called for the opponent (since we don't have their imu)
 * Updates the position of the robot to the given position.
 * @param blob - the MotionBlob to update to
*/
void RobotTracker::UpdateVisionOnly(MotionBlob& blob, cv::Mat& frame)
{
    // derive velocity using the visual information
    cv::Point2f visualVelocity = (blob.center - position) / lastUpdate.getElapsedTime();

    cv::Point2f weightedAverageVelocity = visualVelocity * NEW_VELOCITY_WEIGHT + velocity * (1 - NEW_VELOCITY_WEIGHT);
    // use the blob's center for the visual position
    cv::Point2f visualPosition = blob.center;
    // update using the visual information
    update(visualPosition, weightedAverageVelocity);

    // update our angle using the velocity
    UpdateAngle();
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
    update(newPosition, imuData.velocity);
}

void RobotTracker::update(cv::Point2f position, cv::Point2f velocity)
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