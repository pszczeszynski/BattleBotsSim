#include "RobotTelemetryWidget.h"
#include <vector>
#include <algorithm>
#include "../RobotController.h"
#include "imgui_internal.h"
#include "GraphWidget.h"
#include "UIUtilities.h"
#include "ColorScheme.h"

RobotTelemetryWidget::RobotTelemetryWidget()
{
}

void DrawCANDataTable()
{
    // add Separator
    ImGui::Separator();
    CANData data = RobotController::GetInstance().GetRobotLink().GetLastCANMessage().canData;

    // Increase the font size
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 10)); // Increase spacing for bigger text
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 10)); // Increase padding for bigger text

    // Center the text
    CenterText("Robot CAN Data");

    ImGui::PopStyleVar(2);

    const char *rowLabels[] = {"L Drive", "R Drive", "F Weapon", "B Weapon", "Flippy Boi"};

    if (ImGui::BeginTable("table1", 6, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable))
    {
        // add column headers
        ImGui::TableSetupColumn("");
        ImGui::TableSetupColumn("Voltage");
        ImGui::TableSetupColumn("Current");
        ImGui::TableSetupColumn("ESC Temperature");
        ImGui::TableSetupColumn("Motor Temperature");
        ImGui::TableSetupColumn("RPM");
        ImGui::TableHeadersRow();

        for (int i = 0; i < 5; i++)
        {
            ImGui::TableNextRow();

            // Check if motor data is valid (non-zero temperature indicates active motor)
            if (data.motorVoltage[i] == 0)
            {
                // Motor not active - set dark background
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, 
                    IM_COL32(ColorScheme::BG_DARK.x * 255, ColorScheme::BG_DARK.y * 255, ColorScheme::BG_DARK.z * 255, 255));
            }
            else if (data.escFETTemp[i] > 80 || data.motorVoltage[i] <= 57) // data.motorTemp[i] > 90 || 
            {
                // Set row color to error red
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, 
                    IM_COL32(ColorScheme::STATUS_ERROR.x * 255, ColorScheme::STATUS_ERROR.y * 255, ColorScheme::STATUS_ERROR.z * 255, 255));
            }
            else if (data.escFETTemp[i] > 60) // || data.motorTemp[i] > 60
            {
                // Set row color to warning orange
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, 
                    IM_COL32(ColorScheme::STATUS_WARNING.x * 255, ColorScheme::STATUS_WARNING.y * 255, ColorScheme::STATUS_WARNING.z * 255, 255));
            }
            else
            {
                // Set row color to success green
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, 
                    IM_COL32(ColorScheme::STATUS_SUCCESS.x * 255, ColorScheme::STATUS_SUCCESS.y * 255, ColorScheme::STATUS_SUCCESS.z * 255, 255));
            }

            // Your existing code to set row data
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%s", rowLabels[i]);
            ImGui::Separator();

            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%d V", static_cast<int>(data.motorVoltage[i]));
            ImGui::Separator();

            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%d Amps", static_cast<int>(data.motorCurrent[i]));
            ImGui::Separator();

            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%d °C", static_cast<int>(data.escFETTemp[i]));
            ImGui::Separator();

            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%d °C", static_cast<int>(data.motorTemp[i]));
            ImGui::Separator();

            ImGui::TableSetColumnIndex(5);
            ImGui::Text("%d RPM", (int)(static_cast<int>(data.motorERPM[i]) * ((int)ERPM_FIELD_SCALAR) * ERPM_TO_RPM));
            ImGui::Separator();
        }
        ImGui::EndTable();
    }
}

// Make sure MAX_RPM and MAX_POWER are defined
#define MAX_POWER 1.0

void DrawEnhancedSpinnerRPM(const char* label, float targetPower, int currRPM, const ImVec4& primaryColor, const ImVec4& accentColor)
{
    ImGui::BeginGroup();
    
    // Spinner label with enhanced styling
    ImGui::PushStyleColor(ImGuiCol_Text, primaryColor);
    ImGui::Text("%s", label);
    ImGui::PopStyleColor();
    
    ImGui::SameLine();
    
    // RPM value with large, bold text
    ImGui::PushStyleColor(ImGuiCol_Text, accentColor);
    ImGui::Text("%d RPM", currRPM);
    ImGui::PopStyleColor();
    
    ImGui::Spacing();
    
    // Calculate percentages
    float rpmPercentage = static_cast<float>(currRPM) / MAX_WEAPON_RPM;
    float powerPercentage = static_cast<float>(targetPower) / MAX_POWER;
    
    // Clamp percentages
    rpmPercentage = std::clamp(rpmPercentage, 0.0f, 1.0f);
    powerPercentage = std::clamp(powerPercentage, 0.0f, 1.0f);
    
    // Create a custom progress bar with enhanced visuals
    ImVec2 barSize = ImVec2(-1, 25); // Full width, taller height
    
    // Get the current item rect for proper positioning
    ImVec2 min = ImGui::GetItemRectMin();
    ImVec2 max = ImGui::GetItemRectMax();
    
    // Calculate the actual bar position and size
    ImVec2 pos = ImGui::GetCursorScreenPos();
    float availableWidth = ImGui::GetContentRegionAvail().x;
    ImVec2 endPos = ImVec2(pos.x + availableWidth, pos.y + barSize.y);
    
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    
    // Check if spinner is active (target power > 0)
    bool isActive = targetPower > 0.01f;
    
    // Draw background
    draw_list->AddRectFilled(pos, endPos, ImGui::ColorConvertFloat4ToU32(ColorScheme::BG_MEDIUM), 6.0f);
    
    // Draw enhanced outline with pulsing gradient when active
    if (isActive) {
        // Create pulsing outline with rotating radial gradient
        float time = ImGui::GetTime();
        float pulseIntensity = 0.5f + 0.5f * sin(time * 3.0f); // Smooth pulsing
        
        // Create whiter, more vibrant outline colors
        ImVec4 outlineColor1 = ImVec4(
            0.9f + 0.1f * pulseIntensity,  // Very white with slight pulsing
            0.9f + 0.1f * pulseIntensity,
            0.9f + 0.1f * pulseIntensity,
            0.9f + 0.1f * pulseIntensity
        );
        
        ImVec4 outlineColor2 = ImVec4(
            primaryColor.x + 0.4f * pulseIntensity,  // Enhanced primary color
            primaryColor.y + 0.4f * pulseIntensity,
            primaryColor.z + 0.4f * pulseIntensity,
            0.9f + 0.1f * pulseIntensity
        );
        
        // Draw radial gradient outline that rotates around the bar
        const int outlineSegments = 48; // More segments for smoother radial effect
        for (int i = 0; i < outlineSegments; i++) {
            float t = i / (float)outlineSegments;
            float rotationOffset = time * 1.5f; // Rotation speed
            float radialT = fmod(t + rotationOffset, 1.0f);
            
            // Create radial pattern using sine wave for smooth transitions
            float radialIntensity = 0.5f + 0.5f * sin(radialT * 2.0f * M_PI);
            
            // Interpolate between outline colors with radial pattern
            ImVec4 segmentColor = ImVec4(
                outlineColor1.x + (outlineColor2.x - outlineColor1.x) * radialIntensity,
                outlineColor1.y + (outlineColor2.y - outlineColor1.y) * radialIntensity,
                outlineColor1.z + (outlineColor2.z - outlineColor1.z) * radialIntensity,
                outlineColor1.w + (outlineColor2.w - outlineColor1.w) * radialIntensity
            );
            
            // Calculate segment positions for the outline
            float segmentWidth = availableWidth / outlineSegments;
            ImVec2 segStart(pos.x + i * segmentWidth, pos.y);
            ImVec2 segEnd(pos.x + (i + 1) * segmentWidth, pos.y);
            
            // Draw top edge with radial intensity
            float lineThickness = 2.0f + 2.0f * radialIntensity; // Vary thickness based on radial pattern
            draw_list->AddLine(segStart, segEnd, ImGui::ColorConvertFloat4ToU32(segmentColor), lineThickness);
            
            // Draw bottom edge with radial intensity
            ImVec2 segStartBottom(pos.x + i * segmentWidth, endPos.y);
            ImVec2 segEndBottom(pos.x + (i + 1) * segmentWidth, endPos.y);
            draw_list->AddLine(segStartBottom, segEndBottom, ImGui::ColorConvertFloat4ToU32(segmentColor), lineThickness);
            
            // Draw left and right edges for first and last segments
            if (i == 0) {
                draw_list->AddLine(segStart, segStartBottom, ImGui::ColorConvertFloat4ToU32(segmentColor), lineThickness);
            }
            if (i == outlineSegments - 1) {
                draw_list->AddLine(segEnd, segEndBottom, ImGui::ColorConvertFloat4ToU32(segmentColor), lineThickness);
            }
        }
    } else {
        // Normal border when inactive
        draw_list->AddRect(pos, endPos, ImGui::ColorConvertFloat4ToU32(ColorScheme::BORDER), 6.0f, 0, 2.0f);
    }
    
    // Draw RPM progress bar with gradient effect
    if (rpmPercentage > 0.0f) {
        ImVec2 progressEnd = ImVec2(pos.x + availableWidth * rpmPercentage, endPos.y);
        
        // Create gradient effect
        const int segments = 20;
        for (int i = 0; i < segments; i++) {
            float t = i / (float)segments;
            ImVec2 start(pos.x + availableWidth * rpmPercentage * t, pos.y);
            ImVec2 end(pos.x + availableWidth * rpmPercentage * (t + 1.0f/segments), endPos.y);
            
            // Interpolate color from primary to accent
            ImVec4 segmentColor = ImVec4(
                primaryColor.x + (accentColor.x - primaryColor.x) * t,
                primaryColor.y + (accentColor.y - primaryColor.y) * t,
                primaryColor.z + (accentColor.z - primaryColor.z) * t,
                primaryColor.w
            );
            
            draw_list->AddRectFilled(start, end, ImGui::ColorConvertFloat4ToU32(segmentColor), 4.0f);
        }
    }
    
    // Draw target power indicator line
    if (powerPercentage > 0.0f) {
        float targetX = pos.x + availableWidth * powerPercentage;
        ImVec2 targetStart(targetX, pos.y);
        ImVec2 targetEnd(targetX, endPos.y);
        
        // Draw target line with glow effect
        draw_list->AddLine(targetStart, targetEnd, ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 1.0f, 0.8f)), 3.0f);
        draw_list->AddLine(targetStart, targetEnd, ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 1.0f, 0.4f)), 5.0f);
        
        // Draw target marker
        float markerSize = 8.0f;
        draw_list->AddCircleFilled(ImVec2(targetX, pos.y - markerSize/2), markerSize, ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 1.0f, 0.9f)));
        draw_list->AddCircle(ImVec2(targetX, pos.y - markerSize/2), markerSize, ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 1.0f, 0.6f)), 0, 2.0f);
    }
    
    // Draw percentage text overlay
    char percentageText[16];
    snprintf(percentageText, sizeof(percentageText), "%.1f%%", rpmPercentage * 100.0f);
    ImVec2 textSize = ImGui::CalcTextSize(percentageText);
    ImVec2 textPos(pos.x + (availableWidth - textSize.x) * 0.5f, pos.y + (barSize.y - textSize.y) * 0.5f);
    
    // Text shadow for better readability
    draw_list->AddText(ImVec2(textPos.x + 1, textPos.y + 1), ImGui::ColorConvertFloat4ToU32(ImVec4(0.0f, 0.0f, 0.0f, 0.5f)), percentageText);
    draw_list->AddText(textPos, ImGui::ColorConvertFloat4ToU32(ColorScheme::TEXT_PRIMARY), percentageText);
    
    // Update cursor position using ImGui layout
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + barSize.y + 5);
    
    ImGui::EndGroup();
}

void DrawRPMProgressBar(float targetPower, int currRPM)
{
    // Calculate the percentages
    float rpmPercentage = static_cast<float>(currRPM) / MAX_WEAPON_RPM;
    float powerPercentage = static_cast<float>(targetPower) / MAX_POWER;

    // Choose the progress bar color
    ImU32 barColor = currRPM < static_cast<int>(powerPercentage * MAX_WEAPON_RPM) * 0.9 ? IM_COL32(255, 0, 0, 255) : IM_COL32(0, 255, 0, 255);

    // Draw the progress bar
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, barColor);
    ImGui::ProgressBar(rpmPercentage, ImVec2(-1, 0), ""); // '-1' for full width, '0' for default height
    ImGui::PopStyleColor();

    // Calculate the position for the target power line
    ImVec2 min = ImGui::GetItemRectMin();
    ImVec2 max = ImGui::GetItemRectMax();
    float powerLineX = min.x + (max.x - min.x) * powerPercentage;

    // Draw the target power line
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    window->DrawList->AddLine(ImVec2(powerLineX, min.y), ImVec2(powerLineX, max.y), IM_COL32(255, 0, 255, 255), 2.0f);
}

void DrawRadioData()
{
    // get latest message
    RadioData data = RobotController::GetInstance().GetRobotLink().GetLastRadioMessage().radioData;

    // Increase the font size
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 10)); // Increase spacing for bigger text
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 10)); // Increase padding for bigger text

    // Center the text
    CenterText("Radio Data");

    ImGui::PopStyleVar(2);

    const char* rowLabels[] = { "Average Delay (ms)", "Max Delay (ms)", "Invalid Packets per 10", "Movement", "Turn", "Front Weapon", "Back Weapon" };

    // 2 columns (1 for label, 1 for value)
    
    if (ImGui::BeginTable("table2", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable))
    {
        for (int i = 0; i < 7; i++)
        {
            ImGui::TableNextRow();
            // Set the row label
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%s", rowLabels[i]);
            ImGui::Separator();

            ImGui::TableSetColumnIndex(1);
            switch (i)
            {
            case 0:
                ImGui::Text("%d", (int) data.averageDelayMS);
                break;
            case 1:
                ImGui::Text("%d", (int) data.maxDelayMS);
                break;
            case 2:
                ImGui::Text("%d", (int) data.invalidPackets);
                break;
            case 3:
                ImGui::Text("%f", data.movement);
                break;
            case 4:
                ImGui::Text("%f", data.turn);
                break;
            case 5:
                ImGui::Text("%f", data.frontWeaponPower);
                break;
            case 6:
                ImGui::Text("%f", data.backWeaponPower);
                break;
            }
            ImGui::Separator();
        }
        ImGui::EndTable();
    }
}

void DrawBoardTelemetryData()
{
    // get latest message
    BoardTelemetryData data = RobotController::GetInstance().GetRobotLink().GetLastBoardTelemetryMessage().boardTelemetryData;

    // Increase the font size
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 10)); // Increase spacing for bigger text
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 10)); // Increase padding for bigger text

    ImGui::Spacing();
    CenterText("Power Telemetry");

    ImGui::PopStyleVar(2);

    const char* powerRowLabels[] = { "Tx/Rx", "VBat", "5V volt", "5V amp", "3v3 Volt", "Temp"};
    if (ImGui::BeginTable("table3", 3, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable))
    {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("%s", powerRowLabels[0]);
        ImGui::Separator();
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%s", "Tx");
        ImGui::Separator();
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%s", "Rx");
        ImGui::Separator();

        for (int i = 1; i < 6; i++)
        {
            ImGui::TableNextRow();
            // Set the row label
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%s", powerRowLabels[i]);
            ImGui::Separator();

            ImGui::TableSetColumnIndex(2);
            switch (i)
            {
            case 1:
                ImGui::Text("%f V", data.voltage_batt);
                break;
            case 2:
                ImGui::Text("%f V", data.voltage_5v);
                break;
            case 3:
                ImGui::Text("%f A", data.current_5v);
                break;
            case 4:
                ImGui::Text("%f V", data.voltage_3v3);
                break;
            case 5:
                ImGui::Text("%f C", data.temperature);
                break;
            }
            ImGui::Separator();
        }
        ImGui::EndTable();
    }
}


void RobotTelemetryWidget::Draw()
{
    ImGui::Begin("Robot Telemetry");

    // ===== ENHANCED SPINNER RPM DISPLAYS AT TOP =====
    ImGui::PushStyleColor(ImGuiCol_Text, ColorScheme::TEXT_PRIMARY);
    CenterText(" WEAPON SYSTEMS ");
    ImGui::PopStyleColor();
    
    ImGui::Spacing();
    ImGui::Spacing();
    
    // Front Spinner (Blue theme)
    ImVec4 frontPrimaryColor = ColorScheme::PRIMARY_BLUE;
    ImVec4 frontAccentColor = ImVec4(0.3f, 0.7f, 1.0f, 1.0f); // Lighter blue accent
    DrawEnhancedSpinnerRPM("FRONT SPINNER", 
                           Weapons::GetInstance().GetFrontWeaponTargetPower(),
                           (int)(Weapons::GetInstance().GetFrontWeaponRPM()), 
                           frontPrimaryColor, frontAccentColor);
    
    ImGui::Spacing();
    ImGui::Spacing();
    
    // Back Spinner (Gold theme)
    ImVec4 backPrimaryColor = ImVec4(1.0f, 0.8f, 0.0f, 1.0f); // Gold
    ImVec4 backAccentColor = ImVec4(1.0f, 0.9f, 0.3f, 1.0f); // Lighter gold accent
    DrawEnhancedSpinnerRPM("BACK SPINNER", 
                           Weapons::GetInstance().GetBackWeaponTargetPower(),
                           (int)(Weapons::GetInstance().GetBackWeaponRPM()), 
                           backPrimaryColor, backAccentColor);
    
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Spacing();
    
    // ===== CAN DATA BELOW =====
    DrawCANDataTable();

    ImGui::Spacing();
    ImGui::Spacing();

    DrawRadioData();
    DrawBoardTelemetryData();

    ImGui::End();
}