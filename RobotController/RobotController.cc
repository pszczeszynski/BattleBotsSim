#include "RobotController.h"
#ifdef LINUX_MODE
#include "ServerSocketLinux.h"
#else
#include "ServerSocket.h"
#endif
#include "RobotStateParser.h"
#include "MathUtils.h"
#include "Vision.h"
#include <opencv2/core.hpp>
int main()
{
    std::cout << "running " << std::endl;

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
        std::string received = socket.receive();
        if (received == "")
        {
            continue;
        }

        RobotState state = RobotStateParser::parse(received);
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

double angle_us_to_follow_point_last = 0;
cv::Point2f opponent_pos_last = cv::Point2f(0,0);
RobotControllerMessage RobotController::loop(RobotState &state)
{
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

    cv::Point2f opponent_velocity = (opponent_pos_curr - opponent_pos_last) / elapsed_time;
    cv::Point2f oppponent_pos_future = opponent_pos_curr + opponent_velocity * LOOK_AHEAD_TIME;

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


    // std::cout << "elapsed_time: " << elapsed_time << std::endl;
    double angle_velocity = clock.getElapsedTime() > 0 ? ((angle_wrap(angle_us_to_follow_point - angle_us_to_follow_point_last)) / elapsed_time) : 0;
    // std::cout << "angle_velocity: " << angle_velocity * TO_DEG << std::endl;
    angle_us_to_follow_point_last = angle_us_to_follow_point;

    angle_us_to_follow_point = angle_wrap(angle_us_to_follow_point + angle_velocity * 80.0 / 360.0);


    // turn towards other robot for now
    response.turn_amount = -std::clamp(angle_us_to_follow_point / MAX_POWER_ANGLE, -1.0, 1.0);

    response.drive_amount = -1;// * (1 - abs(angle_us_to_follow_point / MAX_POWER_ANGLE));

    const double BACKUP_TIME_SECONDS = 1;
    const double WAIT_TO_BACKUP_TIME = 0.5;
    double distToOpponnent = cv::norm(oppponent_pos_future - cv::Point2f(state.robot_position.x, state.robot_position.z));
    // backups logic

    std::cout << "dist to opponent: " << distToOpponnent << " waitToBackupClock: " << waitToBackupClock.getElapsedTime() << std::endl;
    // if close and wait to backup expired, restart wait to backup
    if (distToOpponnent < 2 && waitToBackupClock.getElapsedTime() > WAIT_TO_BACKUP_TIME && !backingUp)
    {
        std::cout << "starting wait to backup" << std::endl;
        waitToBackupClock.markStart();
        backingUp = true;
    }


    if (backingUp)
    {
        if (waitToBackupClock.getElapsedTime() > WAIT_TO_BACKUP_TIME &&
            waitToBackupClock.getElapsedTime() < WAIT_TO_BACKUP_TIME + BACKUP_TIME_SECONDS)
        {
            // backup
            response.drive_amount = 1;
            response.turn_amount = 0;
        }

        if (waitToBackupClock.getElapsedTime() > WAIT_TO_BACKUP_TIME + BACKUP_TIME_SECONDS)
        {
            std::cout << "done backing up" << std::endl;

            backingUp = false;
        }
    }

    return response;
}
