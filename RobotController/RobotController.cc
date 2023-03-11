#include "RobotController.h"
#ifdef LINUX_MODE
#include "ServerSocketLinux.h"
#else
#include "ServerSocket.h"
#endif
#include "RobotStateParser.h"
#include "MathUtils.h"
#include "Vision.h"

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
}

void RobotController::Run()
{
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

RobotControllerMessage RobotController::loop(RobotState &state)
{
    RobotControllerMessage response{0, 0};

    // angle from opponent to us
    double angle_opponent_to_us = angle_between_points(state.opponent_position.x, state.opponent_position.z,
                                                       state.robot_position.x, state.robot_position.z);

    const double FOLLOW_DIST_RAD = 30 * TO_RAD;
    const double ORBIT_RADIUS_M = 0;//2.5;

    double angle_to_target_from_robot = angle_wrap(angle_opponent_to_us + FOLLOW_DIST_RAD);

    double follow_point_x = state.opponent_position.x + cos(angle_to_target_from_robot) * ORBIT_RADIUS_M;
    double follow_point_z = state.opponent_position.z + sin(angle_to_target_from_robot) * ORBIT_RADIUS_M;

    // angle from us to the follow point
    double angle_us_to_follow_point = angle_between_points(state.robot_position.x, state.robot_position.z,
                                                           follow_point_x, follow_point_z);

    angle_us_to_follow_point += angle_wrap(state.robot_orientation * TO_RAD);
    angle_us_to_follow_point = angle_wrap(angle_us_to_follow_point);
    const double max_power_angle = 60.0 * TO_RAD;

    // turn towards other robot for now
    response.turn_amount = -std::clamp(angle_us_to_follow_point / max_power_angle, -1.0, 1.0);
    response.drive_amount = -1;

    return response;
}
