#include "RobotTracker.h"
#include "MathUtils.h"
#include "OpponentProfile.h"
#include "../Communication/Communication.h"
#include "RobotController.h"

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
#define DIST_BETWEEN_ANG_UPDATES_PX 0

// 0 = no change, 1 = full change
#define MOVING_AVERAGE_RATE 1.0

// how fast the robot needs to be moving to update the angle
#define VELOCITY_THRESH_FOR_ANGLE_UPDATE 100 * WIDTH / 720.0

/**
 * @brief updates the angle of the robot using the velocity
*/
Angle RobotTracker::CalcAnglePathTangent()
{
    Angle retAngleRad;

    double posDiff = norm(position - lastPositionWhenUpdatedAngle);
    double velNorm = norm(velocity);
    std::cout << "velNorm: " << velNorm << std::endl;

    if (velNorm < VELOCITY_THRESH_FOR_ANGLE_UPDATE)
    {
        lastPositionWhenUpdatedAngle = position;
        visualAngleValid = false;
        return retAngleRad;
    }

    // get latest message
    RobotMessage& robotMessage = RobotController::GetInstance().GetLatestMessage();
    // get angular velocity from imu
    double imuSpeedRadPerSec = abs(robotMessage.rotationVelocity);
    if (imuSpeedRadPerSec * TO_DEG > 30)
    {
        lastPositionWhenUpdatedAngle = position;
        visualAngleValid = false;
        return retAngleRad;
    }
    
    // if we have travelled a certain distance since the last update
    // and we are moving fast enough
    if (posDiff >= DIST_BETWEEN_ANG_UPDATES_PX)
    {
        // calcualte the change from the last update
        cv::Point2f delta = position - lastPositionWhenUpdatedAngle;
        // update the angle but only by half the change
        retAngleRad = Angle(atan2(delta.y, delta.x));

        // if (Angle(retAngleRad + M_PI - angle) < Angle(retAngleRad - angle))
        // {
        //     retAngleRad = Angle(retAngleRad + M_PI);
        // }

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
#define VISUAL_INFO_WEIGHT 0.05
#define NEW_VISUAL_VELOCITY_WEIGHT 0.5

void RobotTracker::UpdateVisionAndIMU(MotionBlob& blob, cv::Mat& frame)
{
    // retrieve the latest robot message
    RobotMessage& robotMessage = RobotController::GetInstance().GetLatestMessage();
    // get the velocity from the imu
    cv::Point2f imuVelocity = cv::Point2f(robotMessage.velocity.x, robotMessage.velocity.z);

    // predict the new position using just the velocity
    cv::Point2f averageVelocityLastUpdate = (velocity + cv::Point2f(robotMessage.velocity.x, robotMessage.velocity.z)) / 2;
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


    double imuAngle = robotMessage.rotation;
    // set the angle to the imu angle
    double newAngle = angle + imuAngle - lastIMUAngle;

    // calculate angle using the visual information
    double angleVisual = CalcAnglePathTangent();

    // if can use the visual information
    if (visualAngleValid)
    {

        std::cout << "angle visual: " << angleVisual * TO_DEG << std::endl;
        // if we have visual information, use the visual angle
        newAngle = newAngle + angle_wrap(angleVisual - newAngle) * VISUAL_INFO_WEIGHT;
    }

    angle = Angle(newAngle);

    angleVelocity = robotMessage.rotationVelocity;

    // save the last angle
    lastIMUAngle = imuAngle;
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
void RobotTracker::UpdateIMUOnly()
{
    
    // retrieve the latest robot message
    RobotMessage& robotMessage = RobotController::GetInstance().GetLatestMessage();
    cv::Point2f imuVelocity = cv::Point2f(robotMessage.velocity.x, robotMessage.velocity.z);

    // calcualte the average velocity of the last update's frame
    cv::Point2f averageVelocityLastUpdate = (velocity + imuVelocity) / 2;
    // predict the new position
    cv::Point2f newPosition = position + averageVelocityLastUpdate * lastUpdate.getElapsedTime();
    // update normally
    UpdateSetPosAndVel(newPosition, imuVelocity);

    double angleImu = angle + (Angle(robotMessage.rotation) - Angle(lastIMUAngle));

    // set our angle to the imu angle
    angle = Angle(angleImu);
    angleVelocity = robotMessage.rotationVelocity;

    // save the last angle
    lastIMUAngle = angleImu;
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

double RobotTracker::GetAngleVelocity()
{
    return angleVelocity;
}