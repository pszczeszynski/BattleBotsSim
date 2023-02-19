#include "RobotController.h"
#ifdef LINUX_MODE
#include "ServerSocketLinux.h"
#else
#include "ServerSocket.h"
#endif
#include "RobotStateParser.h"
#include "MathUtils.h"

int main()
{
    std::cout << "running " << std::endl;
    RobotController rc{};
    rc.Run();
}

RobotController::RobotController()
    : socket{"11115"},
      cameraFL{"cameraCapture1"},
      cameraFR{"cameraCapture2"},
      vision{}
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


        // vision
        cv::Mat leftCamera = cameraFL.getFrame();
        cv::Mat rightCamera = cameraFR.getFrame();

        std::vector<Point> pointCloud;
        vision.compute3dPointCloud(leftCamera, rightCamera, pointCloud);

        cv::imshow("left", leftCamera);
        cv::imshow("right", rightCamera);
        char c = cv::waitKey(1);
        if (c == 'q')
        {
            break;
        }

        socket.reply_to_last_sender(RobotStateParser::serialize(response));
    }
}

RobotControllerMessage RobotController::loop(RobotState &state)
{
    RobotControllerMessage response{0, 0, {}};

    // angle from opponent to us
    double angle_opponent_to_us = angle_between_points(state.opponent_position.x, state.opponent_position.z,
                                                       state.robot_position.x, state.robot_position.z);

    const double FOLLOW_DIST_RAD = 30 * TO_RAD;
    const double ORBIT_RADIUS_M = 2.5;
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
    response.drive_amount = -1.0;

    return response;
}
