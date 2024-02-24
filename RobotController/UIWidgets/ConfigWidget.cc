#include "ConfigWidget.h"
#include "../RobotConfig.h"
#include "../GuiUtils.h"
#include "UIUtilities.h"
#include "../RobotController.h"
#include "../Odometry/Heuristic1/HeuristicOdometry.h"
#include "../../Communication/Communication.h"

ConfigWidget::ConfigWidget()
{
}


#define MARGIN_GO_TO_POINT_CONFIG 220

void ConfigWidget::Draw()
{
    ImGui::Begin("Go To Point Config");
    SetMaxWidthWithMargin(MARGIN_GO_TO_POINT_CONFIG);
    ImGui::SliderInt("Turn Thresh 1 Deg", &TURN_THRESH_1_DEG, 0, 360);
    ImGui::SliderInt("Turn Thresh 2 Deg", &TURN_THRESH_2_DEG, 0, 360);
    ImGui::SliderInt("Min Turn Power (%)", &MIN_TURN_POWER_PERCENT, 0, 100);
    ImGui::SliderInt("Max Turn Power (%)", &MAX_TURN_POWER_PERCENT, 0, 100);
    ImGui::SliderInt("Scale Down Movement (%)", &SCALE_DOWN_MOVEMENT_PERCENT, 0, 100);
    ImGui::SliderInt("Position Extrapolate (ms)", &POSITION_EXTRAPOLATE_MS, 0, 1000);
    ImGui::SliderInt("Master Move Scale (%)", &MASTER_MOVE_SCALE_PERCENT, 0, 100);
    ImGui::SliderInt("Master Turn Scale (%)", &MASTER_TURN_SCALE_PERCENT, 0, 100);
    EndSetMaxWidthWithMargin();
    ImGui::End();

    ImGui::Begin("Orbit Config");
    SetMaxWidthWithMargin(MARGIN_GO_TO_POINT_CONFIG);
    ImGui::SliderInt("Angle Extrapolate (ms)", &ORBIT_ANGLE_EXTRAPOLATE_MS, 0, 1000);
    ImGui::SliderInt("Orbit Radius", &ORBIT_RADIUS, 0, 1000);
    ImGui::SliderInt("Orbit Radius MovAvg Speed (%)", &ORBIT_RADIUS_MOVAVG_SPEED, 0, 100);
    ImGui::SliderInt("PP Radius", &PURE_PURSUIT_RADIUS, 0, 1000);
    ImGui::SliderInt("Opponent Position Extrap (ms)", &OPPONENT_POSITION_EXTRAPOLATE_MS, 0, 1000);
    ImGui::SliderInt("Go Around Radius", &GO_AROUND_RADIUS, 0, 500);
    EndSetMaxWidthWithMargin();
    ImGui::End();

    
    ImGui::Begin("Vision Config");
    SetMaxWidthWithMargin(MARGIN_GO_TO_POINT_CONFIG); 
    ImGui::SliderInt("Min Robot Blob Size", &MIN_ROBOT_BLOB_SIZE, 0, 1000);
    ImGui::SliderInt("Max Robot Blob Size", &MAX_ROBOT_BLOB_SIZE, 0, 1000);
    ImGui::SliderInt("Min Opponent Blob Size", &MIN_OPPONENT_BLOB_SIZE, 0, 1000);
    ImGui::SliderInt("Max Opponent Blob Size", &MAX_OPPONENT_BLOB_SIZE, 0, 1000);
    ImGui::SliderInt("Motion Low Threshold", &MOTION_LOW_THRESHOLD, 0, 100);
    ImGui::SliderFloat("Min fps", &BLOBS_MIN_FPS, 0, 100);
    EndSetMaxWidthWithMargin();

    
    ImGui::End();

    ImGui::Begin("Heuristic Config");
    SetMaxWidthWithMargin(MARGIN_GO_TO_POINT_CONFIG*2); 

    // Background Management
    HeuristicOdometry& heuristic = RobotController::GetInstance().odometry.GetHeuristicOdometry();

    if (ImGui::Button(  "Auto Lock Us Start Left" )) { heuristic.MatchStart(cv::Point2f(30,350),cv::Point2f(700,350) ); }
    ImGui::SameLine();
    ImGui::Text("     ");
    ImGui::SameLine();
    if (ImGui::Button(  "Auto Lock Us Start Right" )) { heuristic.MatchStart(cv::Point2f(30,350),cv::Point2f(700,350) ); }
    ImGui::Text("BACKGROUND:  ");
    ImGui::SameLine();
    if (ImGui::Button(  "Load Bg" )) { heuristic.reinit_bg = true; }
    ImGui::SameLine();
    if (ImGui::Button(  "Save Bg" )) { heuristic.save_background = true; }
    ImGui::SameLine();
    ImGui::Text("  BG Averaging:");
    ImGui::SameLine();
    ImGui::PushItemWidth(250); ImGui::SliderInt("##BGAveraging", &HEU_BACKGROUND_AVGING, 5, 200); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("  Obj Decay:");
    ImGui::SameLine();
    ImGui::PushItemWidth(250); ImGui::SliderInt("##UntrackedDecay", &HEU_UNTRACKED_MOVING_BLOB_AVGING, 50, 400); ImGui::PopItemWidth();

    // Foreground Management
    ImGui::Text("FOREGROUND:  ");
    ImGui::SameLine();
    ImGui::Text("FG Min Delta:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##FGThreshold", &HEU_FOREGROUND_THRESHOLD, 0, 30); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("FG Min Ratio:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##FGRatio", &HEU_FOREGROUND_RATIO, 0, 30); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("  FG Min Size:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##FGMinSize", &HEU_FOREGROUND_MINSIZE, 5, 70); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("  FG Blur Size:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##FGBlur", &HEU_FOREGROUND_BLURSIZE, 5, 30); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("  BBox Buffer:");
    ImGui::SameLine();
    ImGui::PushItemWidth(50); ImGui::SliderInt("##FGBuffer", &HEU_FOREGROUND_BUFFER, 0, 10); ImGui::PopItemWidth();

    // Tracked Robot Management
    ImGui::Text("TRACKING:  ");
    ImGui::SameLine();
    ImGui::Text("Pos Centering Speed:");
    ImGui::SameLine();
    ImGui::PushItemWidth(200); ImGui::SliderInt("##TRPosSpeed", &HEU_POSITION_TO_CENTER_SPEED, 1, 400); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("Vel Averaging:");
    ImGui::SameLine();
    ImGui::PushItemWidth(200); ImGui::SliderInt("##TRVelAvg", &HEU_VELOCITY_AVERAGING, 1, 100); ImGui::PopItemWidth();
        
    // Tracked Robot Management
    ImGui::Text("INTERNALS:  ");
    ImGui::SameLine();
    ImGui::Text("Num Of Processors:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##RobProcessors", &HEU_ROBOT_PROCESSORS, 0, 32); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Checkbox(":Show BG Mat   ", &heuristic.show_bg_mat);
    ImGui::SameLine();
    ImGui::Checkbox(":Show FG Mat     ", &heuristic.show_fg_mat);
    ImGui::SameLine();
    ImGui::Checkbox(":Show Tracking Mat     ", &heuristic.show_track_mat);
    ImGui::SameLine();
    ImGui::Checkbox(":Show Stats", &heuristic.show_stats);
    ImGui::End();

    ImGui::Begin("Radio Config");
    SetMaxWidthWithMargin(MARGIN_GO_TO_POINT_CONFIG);
    // add edit number text box for MIN_INTER_SEND_TIME_MS. Center the number
    ImGui::InputInt("Min Inter Send Time (ms)", &MIN_INTER_SEND_TIME_MS);
    
    // radio channel
    ImGui::InputInt("Radio Channel", &RADIO_CHANNEL);

    // button to set radio channel to 4
    if (ImGui::Button("Teensy #1"))
    {
        RADIO_CHANNEL = TEENSY_RADIO_1;
    }

    ImGui::SameLine();

    if (ImGui::Button("Teensy #2"))
    {
        RADIO_CHANNEL = TEENSY_RADIO_2;
    }

    ImGui::SameLine();

    if (ImGui::Button("Teensy #3"))
    {
        RADIO_CHANNEL = TEENSY_RADIO_3;
    }

    ImGui::End();

    ImGui::Begin("Config File");
    // SetMaxWidthWithMargin(MARGIN_GO_TO_POINT_CONFIG);

    // filename
    InputTextWithString("Save File Name", SAVE_FILE_NAME);

    // add save button
    if (ImGui::Button("Save Config"))
    {
        saveGlobalVariablesToFile(SAVE_FILE_NAME);
    }

    // add load button
    if (ImGui::Button("Load Config"))
    {
        loadGlobalVariablesFromFile(SAVE_FILE_NAME);
    }

    // EndSetMaxWidthWithMargin();
    ImGui::End();
}