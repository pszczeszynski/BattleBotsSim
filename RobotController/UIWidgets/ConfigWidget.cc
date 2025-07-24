#include "ConfigWidget.h"
#include "../RobotConfig.h"
#include "../GuiUtils.h"
#include "UIUtilities.h"
#include "../RobotController.h"
#include "../Odometry/Heuristic1/HeuristicOdometry.h"
#include "../../Common/Communication.h"
#include "FieldWidget.h"



ConfigWidget::ConfigWidget()
{

}


#define MARGIN_GO_TO_POINT_CONFIG 220

void ConfigWidget::Draw()
{
    HeuristicOdometry& heuristic = RobotController::GetInstance().odometry.GetHeuristicOdometry();


    ImGui::Begin("Kill Config");
    // button for LEAD_WITH_BAR
    if (ImGui::Button("Lead with Bar or Disk?"))
    {
        LEAD_WITH_BAR = !LEAD_WITH_BAR;
    }
    ImGui::SameLine();
    ImGui::Text(LEAD_WITH_BAR ? "BAR" : "DISK");

    ImGui::SliderInt("Robot pos extrap (ms)", &POSITION_EXTRAPOLATE_MS, 0, 1000);
    ImGui::SliderInt("Opponent pos extrap (ms)", &OPPONENT_POSITION_EXTRAPOLATE_MS_KILL, 0, 1000);
    ImGui::SliderInt("Max extrap time (ms)", &MAX_OPP_EXTRAP_MS_KILL, 0, 2000);
    ImGui::End();



    ImGui::Begin("Vision Config");
    SetMaxWidthWithMargin(MARGIN_GO_TO_POINT_CONFIG);
    ImGui::SliderInt("Min Robot Blob Size", &MIN_ROBOT_BLOB_SIZE, 0, 1000);
    ImGui::SliderInt("Max Robot Blob Size", &MAX_ROBOT_BLOB_SIZE, 0, 1000);
    ImGui::SliderInt("Min Opponent Blob Size", &MIN_OPPONENT_BLOB_SIZE, 0, 1000);
    ImGui::SliderInt("Max Opponent Blob Size", &MAX_OPPONENT_BLOB_SIZE, 0, 1000);
    ImGui::SliderInt("Motion Low Threshold", &MOTION_LOW_THRESHOLD, 0, 100);
    ImGui::SliderInt("Matching Dist Thresh", &BLOB_MATCHING_DIST_THRESHOLD, 0, 100);
    ImGui::SliderFloat("Min fps", &BLOBS_MIN_FPS, 0, 100);
    EndSetMaxWidthWithMargin();

    
    ImGui::End();

    ImGui::Begin("Heuristic Config");
    SetMaxWidthWithMargin(MARGIN_GO_TO_POINT_CONFIG*2); 

 

    ImGui::Text("BACKGROUND:  ");
    ImGui::SameLine();

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
    if (ImGui::Button("Curr Frame To BG"))
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
    ImGui::PushItemWidth(100); ImGui::SliderInt("##FGThreshold", &HEU_FOREGROUND_THRESHOLD, 0, 90); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("FG Min Ratio:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##FGRatio", &HEU_FOREGROUND_RATIO, 0, 50); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("  FG Min Size:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##FGMinSize", &HEU_FOREGROUND_MINSIZE, 5, 70); ImGui::PopItemWidth();
      ImGui::SameLine();
    ImGui::Text("  FG Max Size:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##FGMaxSize", &HEU_FOREGROUND_MAXSIZE, 5, 500); ImGui::PopItemWidth();
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
    ImGui::SameLine();
    ImGui::Text("Vel to Angle Min Speed:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##TRVelMinSpeed", &HEU_VEL_TO_ANGLE_MIN, 1, 100); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("Vel To Angle Factor:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##TRVelAvgFactor", &HEU_VEL_TO_ANGLE_K, 1, 100); ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("Rot Gain:");
    ImGui::PushItemWidth(100); ImGui::SliderFloat("##HEUROTGAIN", &HEU_ROT_GAIN, 1.0, 2.0); ImGui::PopItemWidth();


    // Tracked Robot Management
    ImGui::Text("INTERNALS:  ");
    ImGui::SameLine();
    ImGui::Text("Num Of Processors:");
    ImGui::SameLine();
    ImGui::PushItemWidth(100); ImGui::SliderInt("##RobProcessors", &HEU_ROBOT_PROCESSORS, 1, 32); ImGui::PopItemWidth();

    ImGui::Checkbox(":BG Mat ", &heuristic.show_bg_mat);
    ImGui::SameLine();
    ImGui::Checkbox(":FG Mat  ", &heuristic.show_fg_mat);
    ImGui::SameLine();
    ImGui::Checkbox(":Tracking  ", &heuristic.show_track_mat);
    ImGui::SameLine();
    ImGui::Checkbox(":Stats ", &heuristic.show_stats);
    ImGui::SameLine();
    ImGui::Checkbox(":Save Video Debug", &heuristic.save_to_video_match_debug);
    ImGui::SameLine();
    ImGui::Checkbox(":Save Video Track", &heuristic.save_to_video_output);
    ImGui::SameLine();
    ImGui::Checkbox(":Log Odometry", &LOG_ODOMETRY_DATA);
    ImGui::SameLine();
    ImGui::Checkbox(":Extract Tuning", &heuristic.show_tuning);
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

        ImGui::SameLine();

        if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_4)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        else if (RADIO_CHANNEL == TEENSY_RADIO_4)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));
        }

        if (ImGui::Button("Tx1->Rx4"))
        {
            RADIO_CHANNEL = TEENSY_RADIO_4;
        }

        if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_4)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        else if (RADIO_CHANNEL == TEENSY_RADIO_4)
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

        ImGui::SameLine();

        if (RADIO_CHANNEL == TEENSY_RADIO_4)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        else if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_4)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));
        }

        if (ImGui::Button("Tx2->Rx4"))
        {
            SECONDARY_RADIO_CHANNEL = TEENSY_RADIO_4;
        }

        if (RADIO_CHANNEL == TEENSY_RADIO_4)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        else if (SECONDARY_RADIO_CHANNEL == TEENSY_RADIO_4)
        {
            ImGui::PopStyleColor();
        }
    }

    if(ImGui::Button("Force IMU Calibration"))
    {
        RESET_IMU = true;
    }

    if(FUSE_IMU)
    {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.7f, 0.0f, 1.0f));
    }
    else
    {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.0f, 0.0f, 1.0f));
    }

    if(ImGui::Button("Fuse IMU"))
    {
        FUSE_IMU = !FUSE_IMU;
    }
    
    ImGui::PopStyleColor();

    
    ImGui::SameLine();

    if(INTEGRATE_GYRO_VEL)
    {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.7f, 0.0f, 1.0f));
    }
    else
    {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.0f, 0.0f, 1.0f));
    }

    if(ImGui::Button("Use IMU Vel"))
    {
        INTEGRATE_GYRO_VEL = !INTEGRATE_GYRO_VEL;
    }

    

    
    ImGui::PopStyleColor();

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