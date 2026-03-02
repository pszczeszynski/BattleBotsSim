#include "StatusIndicatorWidget.h"
#include "../RobotController.h"
#include "../Input/Gamepad.h"
#include "ColorScheme.h"
#include "imgui.h"

StatusIndicatorWidget::StatusIndicatorWidget() {}

static ImU32 ImVec4ToU32(const ImVec4& col)
{
    return ImGui::ColorConvertFloat4ToU32(col);
}

static void DrawGlowyStatusCircle(ImDrawList* drawList, const ImVec2& center, float radius,
    const ImVec4& baseColor, const char* label)
{
    // Outer glow layers - progressively larger, more transparent
    const int glowLayers = 4;
    for (int i = glowLayers; i >= 1; --i)
    {
        float glowRadius = radius + (float)i * 4.0f;
        float alpha = 0.15f * (1.0f - (float)i / (glowLayers + 1));
        ImVec4 glowColor(baseColor.x, baseColor.y, baseColor.z, alpha);
        drawList->AddCircleFilled(center, glowRadius, ImVec4ToU32(glowColor), 32);
    }

    // Main filled circle
    drawList->AddCircleFilled(center, radius, ImVec4ToU32(baseColor), 32);

    // Inner bright ring for shine
    ImVec4 brightColor(
        std::min(1.0f, baseColor.x + 0.4f),
        std::min(1.0f, baseColor.y + 0.4f),
        std::min(1.0f, baseColor.z + 0.4f),
        0.9f);
    drawList->AddCircle(center, radius - 1.0f, ImVec4ToU32(brightColor), 32, 2.0f);

    // Shiny highlight - small bright spot at top-left of circle
    ImVec2 highlightOffset(-radius * 0.35f, -radius * 0.35f);
    ImVec2 highlightCenter(center.x + highlightOffset.x, center.y + highlightOffset.y);
    float highlightRadius = radius * 0.35f;
    ImVec4 highlightColor(1.0f, 1.0f, 1.0f, 0.5f);
    drawList->AddCircleFilled(highlightCenter, highlightRadius, ImVec4ToU32(highlightColor), 16);

    // Label below circle
    ImVec2 textSize = ImGui::CalcTextSize(label);
    ImVec2 textPos(center.x - textSize.x * 0.5f, center.y + radius + 4.0f);
    drawList->AddText(textPos, ImVec4ToU32(ColorScheme::TEXT_PRIMARY), label);
}

void StatusIndicatorWidget::Draw()
{
    ImGui::SetNextWindowSize(ImVec2(320, 80), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin("Status", nullptr, ImGuiWindowFlags_NoCollapse))
    {
        ImGui::End();
        return;
    }

    auto& robotController = RobotController::GetInstance();
    auto& robotLink = robotController.GetRobotLink();
    RadioData radioData = robotLink.GetLastRadioMessage().radioData;
    bool txConnected = robotLink.IsTransmitterConnected();
    bool tx2Connected = robotLink.IsSecondaryTransmitterConnected();
    bool gp1Connected = static_cast<XBox&>(robotController.GetGamepad()).IsConnected();
    bool gp2Connected = static_cast<XBox&>(robotController.GetGamepad2()).IsConnected();

    // Determine colors (matching RobotController::DrawStatusIndicators logic)
    auto getRadioColor = [&radioData]() -> ImVec4
    {
        if (radioData.averageDelayMS < 20 && radioData.averageDelayMS >= 0)
            return ColorScheme::STATUS_SUCCESS;
        if (radioData.averageDelayMS < 50 && radioData.averageDelayMS >= 0)
            return ColorScheme::STATUS_WARNING;
        return ColorScheme::STATUS_ERROR;
    };

    ImVec4 radioColor = getRadioColor();
    ImVec4 txColor = txConnected ? ColorScheme::STATUS_SUCCESS : ColorScheme::STATUS_ERROR;
    ImVec4 tx2Color = tx2Connected ? ColorScheme::STATUS_SUCCESS : ColorScheme::STATUS_ERROR;
    ImVec4 gp1Color = gp1Connected ? ColorScheme::STATUS_SUCCESS : ColorScheme::STATUS_ERROR;
    ImVec4 gp2Color = gp2Connected ? ColorScheme::STATUS_SUCCESS : ColorScheme::STATUS_ERROR;

    const float radius = 18.0f;
    const float spacing = 52.0f;
    ImVec2 startPos = ImGui::GetCursorScreenPos();
    startPos.x += spacing * 0.5f;
    startPos.y += radius + 8.0f;

    ImDrawList* drawList = ImGui::GetWindowDrawList();

    struct Indicator { const char* label; ImVec4 color; };
    Indicator indicators[] = {
        {"Radio", radioColor},
        {"Tx", txColor},
        {"Tx2", tx2Color},
        {"GP1", gp1Color},
        {"GP2", gp2Color}
    };

    for (int i = 0; i < 5; ++i)
    {
        ImVec2 center(startPos.x + (float)i * spacing, startPos.y);
        DrawGlowyStatusCircle(drawList, center, radius, indicators[i].color, indicators[i].label);
    }

    // Reserve space for the drawn content
    ImGui::Dummy(ImVec2(spacing * 5, (radius + 24.0f) * 2));

    ImGui::End();
}
