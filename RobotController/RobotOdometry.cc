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
    // get angular velocity from imu
    return RobotController::GetInstance().GetIMUData().rotation;
}

static double GetImuAngleVelocityRadPerSec()
{
    // get angular velocity from imu
    return RobotController::GetInstance().GetIMUData().rotationVelocity;
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
    IMUData imuData = RobotController::GetInstance().GetIMUData();
    // get angular velocity from imu
    double imuSpeedRadPerSec = abs(imuData.rotationVelocity);
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

bool RobotOdometry::_IsValidBlob(MotionBlob &blob)
{
    double blobArea = blob.rect.area();
    double lastVelocityNorm = cv::norm(_lastVelocity);
    bool invalidBlob = blobArea < _lastBlobArea * 0.8 && _numUpdatesInvalid < 10;

    // if the blob is too small and we haven't had too many invalid blobs
    if (invalidBlob)
    {
        // we are invalid, so increment the number of invalid blobs
        _numUpdatesInvalid++;
    }
    else
    {
        // reset the number of invalid blobs
        _numUpdatesInvalid = 0;
        _lastBlobArea = blobArea;
    }

    return !invalidBlob;
}

/**
 * @brief updates with both visual and imu information. this should only be called for our robot (since we have our imu)
 * However, sometimes we don't have the visual information in which we call UpdateIMUOnly
*/
#define FUSE_ANGLE_WEIGHT 0.05

void RobotOdometry::UpdateVisionAndIMU(MotionBlob& blob, cv::Mat& frame)
{
    //////////////////////// POS ////////////////////////
    cv::Point2f visualPos = blob.center;
    cv::Point2f predictedPosition = _position + _lastVelocity * _lastUpdateClock.getElapsedTime();

    bool valid = _IsValidBlob(blob);
    valid = true;

    //////////////////////// VEL ////////////////////////
    cv::Point2f smoothedVisualVelocity = _GetSmoothedVisualVelocity(blob);
    if (!valid)
    {
        smoothedVisualVelocity = _lastVelocity;
    }

    /////////////////////// ANGLE ///////////////////////
    // set the fused angle to the imu angle
    double fusedAngle = _UpdateAndGetIMUAngle();
    // calculate angle using the visual information
    double visualAngle = CalcAnglePathTangent();
    // if can use the visual information and the user presses enter (to realign)
    if (_visualAngleValid && Input::GetInstance().IsKeyPressed(Qt::Key_Control))
    {
        // if we have visual information, use the visual angle
        fusedAngle = InterpolateAngles(Angle(fusedAngle), Angle(visualAngle), FUSE_ANGLE_WEIGHT);
    }

    // update using the weighted average
    _PostUpdate(visualPos, smoothedVisualVelocity, Angle(fusedAngle));
}

/**
 * Called when we only have the imu information. This should only be called for our robot (since we have our imu)
*/
void RobotOdometry::UpdateIMUOnly()
{
    //////////////////////// POS ////////////////////////
    // predict where we are now using the velocity from the last update and the current velocity
    cv::Point2f predictedPosition = _position + _lastVelocity * _lastUpdateClock.getElapsedTime();

    //////////////////////// VEL ////////////////////////
    // just use the last velocity
    cv::Point2f velocity = _lastVelocity;

    //////////////////////// ANGLE ////////////////////////
    // set the angle using just the imu
    Angle angle = Angle(_UpdateAndGetIMUAngle());

    // update normally
    _PostUpdate(_position, velocity, angle);
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
    cv::Point2f predictedPosition = _position + _lastVelocity * _lastUpdateClock.getElapsedTime();
    bool valid = _IsValidBlob(blob);
    if (!valid)
    {
        visualPosition = predictedPosition;
    }

    //////////////////////// VEL ////////////////////////
    cv::Point2f smoothedVisualVelocity = _GetSmoothedVisualVelocity(blob);
    if (!valid)
    {
        smoothedVisualVelocity = _lastVelocity;
    }

    //////////////////////// ANGLE ////////////////////////
    Angle angle = CalcAnglePathTangent();

    if (_visualAngleValid)
    {
        angle = _angle;
    }   

    // update using the visual information
    _PostUpdate(visualPosition, smoothedVisualVelocity, angle);
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
double RobotOdometry::_UpdateAndGetIMUAngle()
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
#define NEW_VISUAL_VELOCITY_TIME_WEIGHT_MS 50
#define NEW_VISUAL_VELOCITY_WEIGHT_DIMINISH_OPPONENT 3
cv::Point2f RobotOdometry::_GetSmoothedVisualVelocity(MotionBlob& blob)
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

    // If this is the opponent, don't extrapolate so much!! => TODO: make this not a hack
    if (this == &RobotOdometry::Opponent())
    {
        weight /= NEW_VISUAL_VELOCITY_WEIGHT_DIMINISH_OPPONENT;
    }

    // interpolate towards the visual velocity so it's not so noisy
    cv::Point2f smoothedVisualVelocity = InterpolatePoints(_lastVelocity, visualVelocity, weight);

    // restart the clock
    _lastVelocityCalcClock.markStart();

    // return the smoothed velocity
    return smoothedVisualVelocity;
}

void RobotOdometry::_PostUpdate(cv::Point2f newPos, cv::Point2f velocity, Angle angle)
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