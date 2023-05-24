#include "RobotController.h"
#include "ServerSocket.h"
#include "RobotStateParser.h"
#include "MathUtils.h"
#include "Vision.h"
#include <opencv2/core.hpp>

int main()
{
    RobotController rc{};
    rc.Run();

    return 0;
}

RobotController::RobotController()
    : socket{"11115"},
      cameraFL{"cameraCaptureFL"},
      cameraFR{"cameraCaptureFR"},
      cameraBL{"cameraCaptureBL"},
      cameraBR{"cameraCaptureBR"},
      cameraLL{"cameraCaptureLL"},
      cameraLR{"cameraCaptureLR"},
      cameraRL{"cameraCaptureRL"},
      cameraRR{"cameraCaptureRR"},
      vision{cameraFL, cameraFR, cameraBL, cameraBR, cameraLL, cameraLR, cameraRL, cameraRR}
{
    waitToBackupClock.markStart();
}

void RobotController::Run()
{
    GameLoop::run = false;
    // receive until the peer closes the connection
    while (true)
    {
        // 1. receive state info from unity
        std::string received = socket.receive();
        if (received == "")
        {
            continue;
        }

        // 2. parse state info
        RobotState state = RobotStateParser::parse(received);
        // 3. run our robot controller loop
        RobotControllerMessage response = loop(state);

        char key = cv::waitKey(1);

        if (!GameLoop::run)
        {
            response.drive_amount = 0;
            response.turn_amount = 0;
        }

        if (GameLoop::backup)
        {
            response.drive_amount = 1;
            response.turn_amount = 0;
        }

        // send the response back to unity (tell it how much to drive and turn)
        socket.reply_to_last_sender(RobotStateParser::serialize(response));

        // calculate delta position from us to the opponent
        cv::Point3f deltaPos = cv::Point3f(state.opponent_position.x, state.opponent_position.y, state.opponent_position.z) -
                               cv::Point3f(state.robot_position.x, state.robot_position.y, state.robot_position.z);
        // rotate it by our rotation
        cv::Point3f deltaPosRelative = rotate_point(deltaPos, -state.robot_orientation * TO_RAD - M_PI / 2);
        // run the pipeline
        vision.runPipeline(deltaPosRelative, cv::Point3f(response.drive_amount, 0, response.turn_amount));
    }
}

/**
 * This is the main robot controller loop. It is called once per frame.
 * @param state The current state of the robot and opponent
 * @return The response to send back to unity
*/
RobotControllerMessage RobotController::loop(RobotState &state)
{
    static cv::Point2f opponent_pos_last = cv::Point2f(0,0);
    static double angle_us_to_follow_point_last = 0;

    RobotControllerMessage response{0, 0};
    // get elapsed time since last update
    double elapsed_time = clock.getElapsedTime();
    clock.markStart();


    cv::Point2f opponent_pos_curr = cv::Point2f(state.opponent_position.x, state.opponent_position.z);
    double LOOK_AHEAD_TIME = 1 * norm(opponent_pos_curr - cv::Point2f(state.robot_position.x, state.robot_position.z)) / 4.0;
    if (LOOK_AHEAD_TIME > 1)
    {
        LOOK_AHEAD_TIME = 1;
    }

    // calculate opponent velocity
    cv::Point2f opponent_velocity = (opponent_pos_curr - opponent_pos_last) / elapsed_time;
    // calculate opponent future position based on velocity and the look ahead time
    cv::Point2f oppponent_pos_future = opponent_pos_curr + opponent_velocity * LOOK_AHEAD_TIME;

    // save opponent position for next time
    opponent_pos_last = opponent_pos_curr;
    // angle from opponent to us
    double angle_opponent_to_us = angle_between_points(oppponent_pos_future.x, oppponent_pos_future.y,
                                                       state.robot_position.x, state.robot_position.z);

    const double FOLLOW_DIST_RAD = 30 * TO_RAD;
    const double ORBIT_RADIUS_M = 0;//2.5;
    const double MAX_POWER_ANGLE = 30.0 * TO_RAD;

    double angle_to_target_from_robot = angle_wrap(angle_opponent_to_us + FOLLOW_DIST_RAD);

    double follow_point_x = oppponent_pos_future.x + cos(angle_to_target_from_robot) * ORBIT_RADIUS_M;
    double follow_point_z = oppponent_pos_future.y + sin(angle_to_target_from_robot) * ORBIT_RADIUS_M;

    // angle from us to the follow point
    double angle_us_to_follow_point = angle_between_points(state.robot_position.x, state.robot_position.z,
                                                           follow_point_x, follow_point_z);

    angle_us_to_follow_point += angle_wrap(state.robot_orientation * TO_RAD);
    angle_us_to_follow_point = angle_wrap(angle_us_to_follow_point);

    double angle_velocity = clock.getElapsedTime() > 0 ? ((angle_wrap(angle_us_to_follow_point - angle_us_to_follow_point_last)) / elapsed_time) : 0;
    angle_us_to_follow_point_last = angle_us_to_follow_point;

    angle_us_to_follow_point = angle_wrap(angle_us_to_follow_point + angle_velocity * 80.0 / 360.0);


    // turn towards other robot for now
    response.turn_amount = -std::clamp(angle_us_to_follow_point / MAX_POWER_ANGLE, -1.0, 1.0);

    response.drive_amount = -1;// * (1 - abs(angle_us_to_follow_point / MAX_POWER_ANGLE));

    const double BACKUP_TIME_SECONDS = 1;
    const double WAIT_TO_BACKUP_TIME = 0.5;
    double distToOpponnent = cv::norm(oppponent_pos_future - cv::Point2f(state.robot_position.x, state.robot_position.z));
    // backups logic

    // if close and wait to backup expired, restart wait to backup
    if (distToOpponnent < 2 && waitToBackupClock.getElapsedTime() > WAIT_TO_BACKUP_TIME && !backingUp)
    {
        // start backing up
        waitToBackupClock.markStart();
        backingUp = true;
    }

    if (backingUp)
    {
        // if delay expired and not too long, backup
        if (waitToBackupClock.getElapsedTime() > WAIT_TO_BACKUP_TIME &&
            waitToBackupClock.getElapsedTime() < WAIT_TO_BACKUP_TIME + BACKUP_TIME_SECONDS)
        {
            // backup
            response.drive_amount = 1;
            response.turn_amount = 0;
        }

        // if backup time expired, stop backing up
        if (waitToBackupClock.getElapsedTime() > WAIT_TO_BACKUP_TIME + BACKUP_TIME_SECONDS)
        {
            backingUp = false;
        }
    }

    return response;
}
