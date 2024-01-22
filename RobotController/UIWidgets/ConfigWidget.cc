#include "ConfigWidget.h"
#include "../RobotConfig.h"
#include "../GuiUtils.h"

ConfigWidget::ConfigWidget()
{
}

void SetMaxWidthWithMargin(int margin)
{
    ImGuiIO &io = ImGui::GetIO();
    // Calculate desired width based on current window width
    ImVec2 availableSpace = ImGui::GetContentRegionAvail();
    float sliderWidth = availableSpace.x - margin;  // you can adjust 'some_offset' based on your needs

    // Set the width of the SliderInt
    ImGui::PushItemWidth(sliderWidth);
}

void EndSetMaxWidthWithMargin()
{
    ImGui::PopItemWidth();
}

void InputTextWithString(std::string label, std::string& value)
{
    char buf[256];
    strcpy(buf, value.c_str());
    ImGui::InputText(label.c_str(), buf, IM_ARRAYSIZE(buf));
    value = std::string(buf);
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
    ImGui::SliderInt("Orbit Radius MovAvg Speed (%)", &ORBIT_RADIUS_MOVAVG_SPEED, 0, 1000);
    ImGui::SliderInt("PP Radius", &PURE_PURSUIT_RADIUS, 0, 1000);
    ImGui::SliderInt("Opponent Position Extrap (ms)", &OPPONENT_POSITION_EXTRAPOLATE_MS, 0, 1000);
    EndSetMaxWidthWithMargin();
    ImGui::End();


    ImGui::Begin("Vision Config");
    SetMaxWidthWithMargin(MARGIN_GO_TO_POINT_CONFIG);
    ImGui::SliderInt("Min Robot Blob Size", &MIN_ROBOT_BLOB_SIZE, 0, 1000);
    ImGui::SliderInt("Max Robot Blob Size", &MAX_ROBOT_BLOB_SIZE, 0, 1000);
    ImGui::SliderInt("Min Opponent Blob Size", &MIN_OPPONENT_BLOB_SIZE, 0, 1000);
    ImGui::SliderInt("Max Opponent Blob Size", &MAX_OPPONENT_BLOB_SIZE, 0, 1000);
    ImGui::SliderInt("Motion Low Threshold", &MOTION_LOW_THRESHOLD, 0, 100);
    EndSetMaxWidthWithMargin();
    ImGui::End();

    ImGui::Begin("Radio Config");
    SetMaxWidthWithMargin(MARGIN_GO_TO_POINT_CONFIG);
    // add edit number text box for MIN_INTER_SEND_TIME_MS. Center the number
    ImGui::InputInt("Min Inter Send Time (ms)", &MIN_INTER_SEND_TIME_MS);


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

    EndSetMaxWidthWithMargin();
    ImGui::End();
}