#include "RobotController.h"
#include "ServerSocket.h"
#include "RobotStateParser.h"
#include "MathUtils.h"

#ifdef ENABLE_VISION
#include "Vision.h"
#include <opencv2/core.hpp>
#endif

int main()
{
    RobotController rc{};
    rc.Run();

    return 0;
}

RobotController::RobotController()
    : socket{"11115"}
#ifdef ENABLE_VISION
      ,overheadCam{"overheadCam"},
      vision{overheadCam}
#endif
{

}

void RobotController::Run()
{
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

#ifdef ENABLE_VISION
        vision.runPipeline();
        char key = cv::waitKey(1);
#endif
        // vision.performOpticalFlow();

        // 3. run our robot controller loop
        RobotControllerMessage response = loop(state);


#ifndef ENABLE_VISION
        // send the response back to unity (tell it how much to drive and turn)
        socket.reply_to_last_sender(RobotStateParser::serialize(response));
#endif
    }
}

/**
 * This is the main robot controller loop. It is called once per frame.
 * @param state The current state of the robot and opponent
 * @return The response to send back to unity
*/
RobotControllerMessage RobotController::loop(RobotState &state)
{
    static Point2f opponent_pos_last = Point2f(0,0);
    static double angle_us_to_follow_point_last = 0;

    RobotControllerMessage response{0, 0};
    // get elapsed time since last update
    double elapsed_time = clock.getElapsedTime();
    clock.markStart();


    Point2f opponent_pos_curr = Point2f(state.opponent_position.x, state.opponent_position.z);
    double LOOK_AHEAD_TIME = 1 * norm(opponent_pos_curr - Point2f(state.robot_position.x, state.robot_position.z)) / 4.0;
    if (LOOK_AHEAD_TIME > 1)
    {
        LOOK_AHEAD_TIME = 1;
    }

    // calculate opponent velocity
    Point2f opponent_velocity = (opponent_pos_curr - opponent_pos_last) / elapsed_time;
    // calculate opponent future position based on velocity and the look ahead time
    Point2f oppponent_pos_future = opponent_pos_curr + opponent_velocity * LOOK_AHEAD_TIME;

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

    return response;
}
