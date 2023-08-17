#include "RobotOdometry.h"
#include "MathUtils.h"
#include "OpponentProfile.h"
#include "../Communication/Communication.h"
#include "RobotController.h"

RobotOdometry::RobotOdometry(cv::Point2f initialPosition) :
    _position{initialPosition},
    _isValid{false}
{
}

/**
 * @brief returns the robot tracker for our robot
*/
RobotOdometry& RobotOdometry::Robot()
{
    static RobotOdometry robot(cv::Point2f(0,0));
    return robot;
}

/**
 * @brief returns the robot tracker for the opponent
*/
RobotOdometry& RobotOdometry::Opponent()
{
    static RobotOdometry opponent(cv::Point2f(0,0));
    return opponent;
}


static double GetImuAngleRad()
{
    // get latest message
    RobotMessage& robotMessage = RobotController::GetInstance().GetLatestMessage();
    // get angular velocity from imu
    return robotMessage.rotation;
}

/**
 * Returns the velocity reported by the imu sensor, but rotated by our angle
*/
cv::Point2f RobotOdometry::GetImuVelocity()
{
    // get latest message
    RobotMessage& robotMessage = RobotController::GetInstance().GetLatestMessage();

    cv::Point2f accelRobotRelative = cv::Point2f(robotMessage.accelX, robotMessage.accelY);
    #define FIELD_LENGTH_METERS 14.63
    accelRobotRelative *= WIDTH / FIELD_LENGTH_METERS;

    // convert to field relative
    cv::Point2f accelFieldRelative = rotate_point(accelRobotRelative, _angle + M_PI);
    
    cv::Point2f newVelocity = _lastVelocity + accelFieldRelative * _lastAccelIntegrateClock.getElapsedTime();

    _lastAccelIntegrateClock.markStart();
    return newVelocity;


    // // get velocity from imu (invert y because top left is 0,0)
    // cv::Point2f imuRobotRelative = cv::Point2f(robotMessage.velocityX, robotMessage.velocityY);

    // #define FIELD_LENGTH_METERS 14.63
    // imuRobotRelative *= WIDTH / FIELD_LENGTH_METERS;

    // double imuAngle = GetImuAngleRad();
    // double ourAngle = _angle;

    // double angleDifferenceRad = angle_wrap(ourAngle - imuAngle);

    // // convert to field relative
    // cv::Point2f imuFieldRelative = rotate_point(imuRobotRelative, -angleDifferenceRad);
    // // return the rotated velocity
    // return imuFieldRelative;
}
static double GetImuAngleVelocityRadPerSec()
{
    // get latest message
    RobotMessage& robotMessage = RobotController::GetInstance().GetLatestMessage();
    // get angular velocity from imu
    return robotMessage.rotationVelocity;
}

// the displacement required to update the angle
#define DIST_BETWEEN_ANG_UPDATES_PX 5

// 0 = no change, 1 = full change
#define MOVING_AVERAGE_RATE 1.0

// how fast the robot needs to be moving to update the angle
#define VELOCITY_THRESH_FOR_ANGLE_UPDATE 100 * WIDTH / 720.0

// max rotational speed to update the angle radians per second
#define MAX_ROTATION_SPEED_TO_ALIGN 250 * TO_RAD

/**
 * @brief updates the angle of the robot using the velocity
*/
Angle RobotOdometry::CalcAnglePathTangent()
{
    Angle retAngleRad;

    double posDiff = norm(_position - _lastPositionWhenUpdatedAngle);
    double velNorm = norm(_lastVelocity);

    // if (velNorm < VELOCITY_THRESH_FOR_ANGLE_UPDATE)
    // {
    //     _lastPositionWhenUpdatedAngle = _position;
    //     _visualAngleValid = false;
    //     return retAngleRad;
    // }

    // get latest message
    RobotMessage& robotMessage = RobotController::GetInstance().GetLatestMessage();
    // get angular velocity from imu
    double imuSpeedRadPerSec = abs(robotMessage.rotationVelocity);
    if (imuSpeedRadPerSec > MAX_ROTATION_SPEED_TO_ALIGN)
    {
        _lastPositionWhenUpdatedAngle = _position;
        _visualAngleValid = false;
        return retAngleRad;
    }
    
    // if we have travelled a certain distance since the last update
    // and we are moving fast enough
    if (posDiff >= DIST_BETWEEN_ANG_UPDATES_PX)
    {
        // calcualte the change from the last update
        cv::Point2f delta = _position - _lastPositionWhenUpdatedAngle;
        // update the angle but only by half the change
        retAngleRad = Angle(atan2(delta.y, delta.x));

        // if the angle is closer to 180 degrees to the last angle
        if (abs(Angle(retAngleRad + M_PI - _angle)) < abs(Angle(retAngleRad - _angle)))
        {
            // add 180 degrees to the angle
            retAngleRad = Angle(retAngleRad + M_PI);
        }

        // add half the change in angle from the imu in the last update (since we calculated the angle at the midpoint)
        retAngleRad = retAngleRad + Angle(_lastVisualAngleValidClock.getElapsedTime() / 2 * GetImuAngleVelocityRadPerSec());
        _lastVisualAngleValidClock.markStart();

        // save the last position
        _lastPositionWhenUpdatedAngle = _position;
        // set the flag to true
        _visualAngleValid = true;
    }
    else
    {
        _visualAngleValid = false;
    }

    return retAngleRad;
}



/**
 * Predicts the current position of the robot using the velocity from last update and curr update
*/
static cv::Point2f PredictCurrPosUsingAvgVelocity(cv::Point2f lastPosition, cv::Point2f lastVelocity, cv::Point2f newVelocity, double timeSinceLastUpdateSeconds)
{
    // get the average velocity of the last update
    cv::Point2f averageVelocityLastUpdate = (lastVelocity + newVelocity) / 2;
    // predict the new position using that average velocity
    cv::Point2f predictedPosition = lastPosition + averageVelocityLastUpdate * timeSinceLastUpdateSeconds;

    return predictedPosition;
}

/**
 * @brief updates with both visual and imu information. this should only be called for our robot (since we have our imu)
 * However, sometimes we don't have the visual information in which we call UpdateIMUOnly
*/
#define VISUAL_LOCATION_INTERP_WEIGHT 1.0
#define FUSE_ANGLE_WEIGHT 0.01

void RobotOdometry::UpdateVisionAndIMU(MotionBlob& blob, cv::Mat& frame)
{
    // put text that we are using vision
    SAFE_DRAW
    cv::putText(drawingImage, "Using Vision and IMU", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2);
    END_SAFE_DRAW

    //////////////////////// POS ////////////////////////
    // predict where we are now using the velocity from the last update and the current velocity
    cv::Point2f imuPos = PredictCurrPosUsingAvgVelocity(_position, _lastVelocity, GetImuVelocity(), _lastUpdateClock.getElapsedTime());
    // do a weighted average of the predicted position and the visual position to smooth out the noise in the visual position
    cv::Point2f fusedPos = InterpolatePoints(imuPos, blob.center, VISUAL_LOCATION_INTERP_WEIGHT);

    //////////////////////// VEL ////////////////////////
    cv::Point2f smoothedVisualVelocity = GetSmoothedVisualVelocity(blob);

    //////////////////////// ANGLE ////////////////////////
    // set the fused angle to the imu angle
    double fusedAngle = UpdateAndGetIMUAngle();
    // calculate angle using the visual information
    double visualAngle = CalcAnglePathTangent();
    // if can use the visual information
    if (_visualAngleValid)
    {
        // if we have visual information, use the visual angle
        fusedAngle = InterpolateAngles(Angle(fusedAngle), Angle(visualAngle), FUSE_ANGLE_WEIGHT);

        // draw arrow at our position and fused angle
        SAFE_DRAW
        cv::arrowedLine(drawingImage, fusedPos, fusedPos + cv::Point2f(100 * cos(fusedAngle), 100 * sin(fusedAngle)), cv::Scalar(255, 0, 255), 2);
        END_SAFE_DRAW
    }

    // update using the weighted average
    PostUpdate(fusedPos, smoothedVisualVelocity, Angle(fusedAngle));
}

/**
 * Called when we only have the imu information. This should only be called for our robot (since we have our imu)
*/
void RobotOdometry::UpdateIMUOnly()
{
    // put text that we are using vision
    SAFE_DRAW
    cv::putText(drawingImage, "Using IMU Only", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
    END_SAFE_DRAW

    //////////////////////// POS ////////////////////////
    // predict where we are now using the velocity from the last update and the current velocity
    cv::Point2f predictedPosition = PredictCurrPosUsingAvgVelocity(_position, _lastVelocity, GetImuVelocity(), _lastUpdateClock.getElapsedTime());

    //////////////////////// VEL ////////////////////////
    cv::Point2f imuVelocity = GetImuVelocity();

    //////////////////////// ANGLE ////////////////////////
    // set the angle using just the imu
    Angle angle = Angle(UpdateAndGetIMUAngle());

    // update normally
    PostUpdate(_position, imuVelocity, angle);
}

/**
 * @brief updates with just visual information. this should only be called for the opponent (since we don't have their imu)
 * Updates the position of the robot to the given position.
 * @param blob - the MotionBlob to update to
*/
void RobotOdometry::UpdateVisionOnly(MotionBlob& blob, cv::Mat& frame)
{
    //////////////////////// POS ////////////////////////
    // use the blob's center for the visual position
    cv::Point2f visualPosition = blob.center;

    //////////////////////// VEL ////////////////////////
    cv::Point2f smoothedVisualVelocity = GetSmoothedVisualVelocity(blob);

    //////////////////////// ANGLE ////////////////////////
    Angle angle = CalcAnglePathTangent();

    if (_visualAngleValid)
    {
        angle = _angle;
    }   

    // update using the visual information
    PostUpdate(visualPosition, smoothedVisualVelocity, angle);
}


/**
 * Allows artificially setting the angle of the robot
 * This is mainly used for manual recalibration.
 * 
 * @return the new angle
*/
double RobotOdometry::UpdateForceSetAngle(double newAngle)
{
    // Set angle to new manual angle value
    _angle = Angle(newAngle);
    return _angle;
}


/**
 * Tracks the change in the imu angle
 * Should be called every time we update using an imu
 * 
 * @return the new angle with the change in angle added
*/
double RobotOdometry::UpdateAndGetIMUAngle()
{
    // 1. compute angle change
    // get the new angle
    double newAngleImuRad = GetImuAngleRad();
    // calculate the change in angle (there are no angle wraps since the imu is continuous)
    double angleChange = newAngleImuRad - _lastIMUAngle;
    // update the last angle
    _lastIMUAngle = newAngleImuRad;


    // 2. compute and save angular velocity for later
    _angleVelocity = GetImuAngleVelocityRadPerSec();


    // 3. return the current angle + change in angle
    return _angle + angleChange;
}

/**
 * Smooths out the visual velocity so it's not so noisy
*/
#define NEW_VISUAL_VELOCITY_TIME_WEIGHT_MS 33
cv::Point2f RobotOdometry::GetSmoothedVisualVelocity(MotionBlob& blob)
{
    if (_lastVelocityCalcClock.getElapsedTime() > 0.1)
    {
        _lastVelocityCalcClock.markStart();
        return cv::Point2f(0, 0);
    }

    // visual velocity
    cv::Point2f visualVelocity = (blob.center - _position) / _lastVelocityCalcClock.getElapsedTime();
    // compute weight for interpolation
    double weight = _lastVelocityCalcClock.getElapsedTime() * 1000 / NEW_VISUAL_VELOCITY_TIME_WEIGHT_MS;
    // interpolate towards the visual velocity so it's not so noisy
    cv::Point2f smoothedVisualVelocity = InterpolatePoints(_lastVelocity, visualVelocity, weight);

    // restart the clock
    _lastVelocityCalcClock.markStart();

    // return the smoothed velocity
    return smoothedVisualVelocity;
}

void RobotOdometry::PostUpdate(cv::Point2f newPos, cv::Point2f velocity, Angle angle)
{
    // update our velocity, position, and angle
    _lastVelocity = velocity;
    _position = newPos;
    _angle = angle;

    _isValid = true;

    // mark this as the last update time
    _lastUpdateClock.markStart();


    // draw the velocity as a bar
    SAFE_DRAW
    // put text for the velocity
    std::stringstream ss;
    ss << "Vel: " << cv::norm(_lastVelocity);
    cv::putText(drawingImage, ss.str(), cv::Point(50, drawingImage.rows - 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    END_SAFE_DRAW
}

/**
 * Allows artificially setting the position and velocity of the robot
 * This is mainly used for manual recalibration.
 * 
 * @param newPos - the new position of the robot
 * @param newVel - the new velocity of the robot
*/
void RobotOdometry::UpdateForceSetPosAndVel(cv::Point2f newPos, cv::Point2f newVel)
{
    // update our velocity and position
    _lastVelocity = newVel;
    _position = newPos;

    // mark this as the last update time
    _lastUpdateClock.markStart();
}

/**
 * @brief getPosition
 * @return the current position of the robot.
*/
cv::Point2f RobotOdometry::GetPosition()
{
    return _position;
}

Angle RobotOdometry::GetAngle()
{
    return _angle;
}

void RobotOdometry::Invalidate()
{
    _lastVelocity = cv::Point2f(0,0);
    _isValid = false;
}

cv::Point2f RobotOdometry::GetVelocity()
{
    return _lastVelocity;
}

double RobotOdometry::GetAngleVelocity()
{
    return _angleVelocity;
}

void RobotOdometry::InvertAngle()
{
    _angle = Angle(_angle + M_PI);
}