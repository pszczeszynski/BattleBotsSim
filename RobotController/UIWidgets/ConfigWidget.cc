#include "ConfigWidget.h"
#include "../RobotConfig.h"
#include "../GuiUtils.h"
#include "UIUtilities.h"
#include "../RobotController.h"
#include "../Odometry/Heuristic1/HeuristicOdometry.h"
#include "../../Common/Communication.h"
#include "FieldWidget.h"

// Define my statics
cv::Point2f ConfigWidget::leftStart = cv::Point2f(0, 0);
cv::Point2f ConfigWidget::rightStart = cv::Point2f(0, 0);

ConfigWidget::ConfigWidget()
{
    // Initialize is here in case the globals have changed
    //leftStart = cv::Point2f(HEU_LEFTSTART_X, HEU_LEFTSTART_Y);
    //rightStart = cv::Point2f(HEU_RIGHTSTART_X, HEU_RIGHTSTART_Y);
}


#define MARGIN_GO_TO_POINT_CONFIG 220

void ConfigWidget::Draw()
{
    ImGui::Begin("Orbit Config");
    SetMaxWidthWithMargin(MARGIN_GO_TO_POINT_CONFIG);
    // button for LEAD_WITH_BAR
    if (ImGui::Button("Lead with Bar or Disk?"))
    {
        LEAD_WITH_BAR = !LEAD_WITH_BAR;
    }
    ImGui::SameLine();
    // text for BAR
    ImGui::Text(LEAD_WITH_BAR ? "BAR" : "DISK");

    ImGui::SliderInt("Angle Extrap switching (ms)", &ORBIT_ANGLE_EXTRAPOLATE_MS, 0, 1000);
    ImGui::SliderInt("Position Extrapolate (ms)", &POSITION_EXTRAPOLATE_MS, 0, 1000);
    ImGui::SliderInt("Orbit Radius", &ORBIT_RADIUS, 0, 1000);
    ImGui::SliderFloat("Orbit Radius MovAvg Blend Time (sec)", &ORBIT_RADIUS_MOVAVG_SPEED, 0, 1.0f);
    ImGui::SliderInt("PP Radius", &PURE_PURSUIT_RADIUS, 0, 1000);
    ImGui::SliderInt("Opponent Position Extrap (ms)", &OPPONENT_POSITION_EXTRAPOLATE_MS, 0, 1000);
    ImGui::SliderInt("Go Around Radius", &GO_AROUND_RADIUS, 0, 500);

    // space
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::SliderFloat("Opponent Weapon Offset", &OPPONENT_WEAPON_OFFSET, 0.0, 150.0);
    ImGui::SliderFloat("Opponent Spiral Start Deg", &OPPONENT_SPIRAL_START_DEG, 0.0, 180.0);
    ImGui::SliderFloat("Opponent Spiral End Deg", &OPPONENT_SPIRAL_END_DEG, 0.0, 180.0);
    ImGui::SliderFloat("Opponent Angle Extrap (ms)", &OPPONENT_ANGLE_EXTRAPOLATE_MS, 0.0, 500.0);
    ImGui::SliderFloat("Preserve momentum factor", &ORBIT_PRESERVE_CURR_ANGLE_WEIGHT, 0.0, 5.0);

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

    // Set leftstart and rightstart to middle of background starting boxes
    leftStart = cv::Point2f((STARTING_LEFT_TL_x+STARTING_LEFT_BR_x)/2, (STARTING_LEFT_TL_y+STARTING_LEFT_BR_y)/2);
    rightStart = cv::Point2f((STARTING_RIGHT_TL_x+STARTING_RIGHT_BR_x)/2, (STARTING_RIGHT_TL_y+STARTING_RIGHT_BR_y)/2);

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.0f, 1.0f, 1.0f));
    if (ImGui::Button("Auto Lock Us Left", ImVec2(200, 50)))
    {
        heuristic.MatchStart(leftStart, rightStart);
    }
    ImGui::PopStyleColor();

    ImGui::SameLine();
    ImGui::Text("     ");
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
    if (ImGui::Button("Auto Lock Us Right", ImVec2(200, 50)))
    {
        heuristic.MatchStart(rightStart, leftStart);
    }
    ImGui::PopStyleColor();

    ImGui::Text("BACKGROUND:  ");
    ImGui::SameLine();
    if (ImGui::Button("Load Start Bg"))
    {
        heuristic.load_start_background = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Load Saved Bg"))
    {
        heuristic.load_background = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Save Bg"))
    {
        heuristic.save_background = true;
    }

    ImGui::SameLine();
    if (ImGui::Button("ReBoot BG"))
    {
        heuristic.set_currFrame_to_bg = true;
    }

    ImGui::SameLine();
    ImGui::Text("  BG Averaging:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100);
    ImGui::SliderInt("##BGAveraging", &HEU_BACKGROUND_AVGING, 5, 400);
    ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("  Obj Decay:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100);
    ImGui::SliderInt("##UntrackedDecay", &HEU_UNTRACKED_MOVING_BLOB_AVGING, 5, 400);
    ImGui::PopItemWidth();
    ImGui::Checkbox(":EN Healing    ", &heuristic.enable_background_healing);
    ImGui::SameLine();
    ImGui::Checkbox(":FORCE HEALING EVEN IF NO BOTS", &heuristic.force_background_averaging);

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
    ImGui::Text("  FG Blur Size:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##FGBlur", &HEU_FOREGROUND_BLURSIZE, 1, 30); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("  BG Blur Size (make odd):");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##BGBlur", &HEU_BACKGROUND_BLURSIZE, 1, 30); ImGui::PopItemWidth();

    ImGui::SameLine();
    ImGui::Text("  Blur Count:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##BlurCount", &HEU_BLUR_COUNT, 0, 10); ImGui::PopItemWidth();

    ImGui::Text("MATCH START ");
    ImGui::SameLine();
    ImGui::Text("  Auto Gamma Corr:");
    ImGui::SameLine();
    ImGui::PushItemWidth(150); ImGui::SliderFloat("##BrightCorr", &HEU_BRIGHTNESS_CORR, 0.5, 1.5); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("  BG Blur Size:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##BGBlurInit", &HEU_BACKGROUND_BLURSIZE_INIT, 1, 30); ImGui::PopItemWidth();
    ImGui::SameLine();

    ImGui::Text("  Blur Count:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##BlurCountInit", &HEU_BLUR_COUNT_INIT, 1, 15); ImGui::PopItemWidth();
    ImGui::SameLine();

    ImGui::Text("FG Min Delta:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##FGThresholdInit", &HEU_FOREGROUND_THRESHOLD_INIT, 0, 40); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Checkbox(":Heal BG", &HEU_HEAL_BG_INIT);


    //ImGui::Text("  BBox Buffer:");
    //ImGui::SameLine();
    //ImGui::PushItemWidth(50); ImGui::SliderInt("##FGBuffer", &HEU_FOREGROUND_BUFFER, 0, 10); ImGui::PopItemWidth();

    // Tracked Robot Management
    ImGui::Text("TRACKING:  ");
    ImGui::SameLine();
    ImGui::Text("Pos Centering Speed:");
    ImGui::SameLine();
    ImGui::PushItemWidth(150); ImGui::SliderInt("##TRPosSpeed", &HEU_POSITION_TO_CENTER_SPEED, 1, 200); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("Vel Averaging:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##TRVelAvg", &HEU_VELOCITY_AVERAGING, 1, 100); ImGui::PopItemWidth();
        
    // Tracked Robot Management
    ImGui::Text("INTERNALS:  ");
    ImGui::SameLine();
    ImGui::Text("Num Of Processors:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##RobProcessors", &HEU_ROBOT_PROCESSORS, 1, 32); ImGui::PopItemWidth();

    ImGui::Checkbox(":Show BG Mat ", &heuristic.show_bg_mat);
    ImGui::SameLine();
    ImGui::Checkbox(":Show FG Mat  ", &heuristic.show_fg_mat);
    ImGui::SameLine();
    ImGui::Checkbox(":Show Tracking  ", &heuristic.show_track_mat);
    ImGui::SameLine();
    ImGui::Checkbox(":Show Stats ", &heuristic.show_stats);
    ImGui::SameLine();
    ImGui::Checkbox(":Save Video Debug", &heuristic.save_to_video_match_debug);
    ImGui::SameLine();
    ImGui::Checkbox(":Save Video Track", &heuristic.save_to_video_output);
    ImGui::End();


    ImGui::Begin("Radio Config");
    SetMaxWidthWithMargin(MARGIN_GO_TO_POINT_CONFIG);
    // add edit number text box for MIN_INTER_SEND_TIME_MS. Center the number
    ImGui::InputInt("Min Inter Send Time (ms)", &MIN_INTER_SEND_TIME_MS);    
    // radio channel
    ImGui::InputInt("Radio Channel", &RADIO_CHANNEL);

    /*static bool disabled = false;
    ImGui::Checkbox("Disable", &disabled);
    if (disabled)
    {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
    }
    [...]
    if (disabled)
    {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();
    }*/

    if (!AUTO_SWITCH_CHANNEL)
    {
        if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_1)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        else if (RADIO_CHANNEL == TEENSY_RADIO_1)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));
        }

        if (ImGui::Button("Tx1->Rx1"))
        {
            RADIO_CHANNEL = TEENSY_RADIO_1;
        }

        if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_1)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        else if (RADIO_CHANNEL == TEENSY_RADIO_1)
        {
            ImGui::PopStyleColor();
        }

        ImGui::SameLine();

        if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_2)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        else if (RADIO_CHANNEL == TEENSY_RADIO_2)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));
        }

        if (ImGui::Button("Tx1->Rx2"))
        {
            RADIO_CHANNEL = TEENSY_RADIO_2;
        }

        if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_2)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        else if (RADIO_CHANNEL == TEENSY_RADIO_2)
        {
            ImGui::PopStyleColor();
        }

        ImGui::SameLine();

        if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_3)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        else if (RADIO_CHANNEL == TEENSY_RADIO_3)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));
        }

        if (ImGui::Button("Tx1->Rx3"))
        {
            RADIO_CHANNEL = TEENSY_RADIO_3;
        }

        if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_3)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        else if (RADIO_CHANNEL == TEENSY_RADIO_3)
        {
            ImGui::PopStyleColor();
        }
    }

    ImGui::InputInt("Secondary Radio Channel", &SECONDARY_RADIO_CHANNEL);
    
    if (!AUTO_SWITCH_CHANNEL)
    {
        if (RADIO_CHANNEL == TEENSY_RADIO_1)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        else if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_1)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.7f, 0.0f, 1.0f));
        }

        if (ImGui::Button("Tx2->Rx1"))
        {
            SECONDARY_RADIO_CHANNEL = TEENSY_RADIO_1;
        }

        if (RADIO_CHANNEL == TEENSY_RADIO_1)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        else if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_1)
        {
            ImGui::PopStyleColor();
        }

        ImGui::SameLine();

        if (RADIO_CHANNEL == TEENSY_RADIO_2)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        else if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_2)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));
        }

        if (ImGui::Button("Tx2->Rx2"))
        {
            SECONDARY_RADIO_CHANNEL = TEENSY_RADIO_2;
        }

        if (RADIO_CHANNEL == TEENSY_RADIO_2)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        else if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_2)
        {
            ImGui::PopStyleColor();
        }

        ImGui::SameLine();

        if (RADIO_CHANNEL == TEENSY_RADIO_3)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        else if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_3)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));
        }

        if (ImGui::Button("Tx2->Rx3"))
        {
            SECONDARY_RADIO_CHANNEL = TEENSY_RADIO_3;
        }

        if (RADIO_CHANNEL == TEENSY_RADIO_3)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        else if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_3)
        {
            ImGui::PopStyleColor();
        }
    }

    // add checkbox for auto switch channel
    ImGui::Checkbox("Auto Switch Channel", &AUTO_SWITCH_CHANNEL);
    // add edit number text box for MAX_AVERAGE_DELAY_MS.
    ImGui::InputInt("Max Average Delay (ms)", &MAX_AVERAGE_DELAY_MS);
    // cooldown
    ImGui::InputInt("Auto Switch Cooldown (ms)", &SWITCH_COOLDOWN_MS);

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