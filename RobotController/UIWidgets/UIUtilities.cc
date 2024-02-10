#include "UIUtilities.h"
#include "imgui.h"

void CenterText(const char *text)
{
    // Center the text
    float windowWidth = ImGui::GetWindowWidth();
    float textWidth = ImGui::CalcTextSize(text).x;
    ImGui::SetCursorPosX((windowWidth - textWidth) / 2);

    ImGui::Text(text);
}

void SetMaxWidthWithMargin(int margin)
{
    ImGuiIO &io = ImGui::GetIO();
    // Calculate desired width based on current window width
    ImVec2 availableSpace = ImGui::GetContentRegionAvail();
    float sliderWidth = availableSpace.x - margin; // you can adjust 'some_offset' based on your needs

    // Set the width of the SliderInt
    ImGui::PushItemWidth(sliderWidth);
}

void EndSetMaxWidthWithMargin()
{
    ImGui::PopItemWidth();
}

void InputTextWithString(std::string label, std::string &value)
{
    char buf[256];
    strcpy(buf, value.c_str());
    ImGui::InputText(label.c_str(), buf, IM_ARRAYSIZE(buf));
    value = std::string(buf);
}
