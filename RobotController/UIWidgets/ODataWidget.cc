#include "ODataWidget.h"
#include <iostream>
#include <filesystem>
#include "../RobotConfig.h"
#include "../Globals.h"
#include "../RobotOdometry.h"
#include "../RobotController.h"

/**
 * @brief Draws the playback interface
*/
void ODataWidget::Draw()
{
    ImGui::Begin("Odometry Data");
    // Get Latest data
    RobotOdometry& odometry = RobotController::GetInstance().odometry;
    currRobot = odometry.Robot();
    currOpponent = odometry.Opponent();

    OdometryData& currOdo =  currRobot;

    ImGui::Text("ODOMETRY DATA:");
    ImGui::Text("  Robot:");
    ImGui::Text("        Pos Valid = %s", (currOdo.robotPosValid) ? "YES" : "NO");
    ImGui::Text("        Angle Valid = %s", (currOdo._robotAngleValid) ? "YES" : "NO");
    ImGui::Text("        Pos   = (%.1f, %.1f)", currOdo.robotPosition.x,currOdo.robotPosition.y);
    ImGui::Text("        Angle = %.1f deg", rad2deg(currOdo._angle));
    ImGui::Text("        Vel =(%.1f, %.1f)", currOdo.robotVelocity.x,currOdo.robotVelocity.y);
    ImGui::Text("        Angle Vel  =%.1f deg/s", rad2deg(currOdo._robotAngleVelocity));

    OdometryData& currOdo2 =  currOpponent;
    ImGui::Text("  Opponent:");
    ImGui::Text("        Pos Valid = %s", (currOdo2.robotPosValid) ? "YES" : "NO");
    ImGui::Text("        Angle Valid = %s", (currOdo2._robotAngleValid) ? "YES" : "NO");
    ImGui::Text("        Pos   = (%.1f, %.1f)", currOdo2.robotPosition.x,currOdo2.robotPosition.y);
    ImGui::Text("        Angle = %.1f deg", rad2deg(currOdo2._angle));
    ImGui::Text("        Vel =(%.1f, %.1f)", currOdo2.robotVelocity.x,currOdo2.robotVelocity.y);
    ImGui::Text("        Angle Vel  =%.1f deg/s", rad2deg(currOdo2._robotAngleVelocity));

    ImGui::End();

}

