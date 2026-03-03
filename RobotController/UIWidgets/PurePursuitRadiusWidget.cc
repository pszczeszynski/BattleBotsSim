#include "PurePursuitRadiusWidget.h"

#include "../RobotConfig.h"
#include "ColorScheme.h"
#include "imgui_internal.h"


constexpr float PP_RAD_WIDGET_WIDTH = 280.0f;

static bool SliderFloatWithFill(const char* label, float* v, float v_min,
                                float v_max, const char* format,
                                const ImVec4& fillColor,
                                const ImVec4& grabColor,
                                const ImVec4& grabActiveColor) {
  ImGuiWindow* window = ImGui::GetCurrentWindow();
  if (window->SkipItems) return false;

  ImGuiContext& g = *ImGui::GetCurrentContext();
  const ImGuiStyle& style = g.Style;
  const ImGuiID id = window->GetID(label);
  const float w = ImGui::GetContentRegionAvail().x;
  if (w <= 0.0f) return false;

  const ImVec2 label_size = ImGui::CalcTextSize(label, NULL, true);
  const ImVec2 frame_max(
      window->DC.CursorPos.x + w,
      window->DC.CursorPos.y + label_size.y + style.FramePadding.y * 2.0f);
  const ImRect frame_bb(window->DC.CursorPos, frame_max);
  const ImRect total_bb(frame_bb.Min, frame_bb.Max);

  ImGui::ItemSize(total_bb, style.FramePadding.y);
  if (!ImGui::ItemAdd(total_bb, id, &frame_bb, ImGuiItemFlags_Inputable))
    return false;

  const bool hovered =
      ImGui::ItemHoverable(frame_bb, id, g.LastItemData.InFlags);
  const bool clicked = hovered && ImGui::IsMouseClicked(0, id);
  const bool make_active = clicked || (g.NavActivateId == id);
  if (make_active) {
    ImGui::SetActiveID(id, window);
    ImGui::SetFocusID(id, window);
    ImGui::FocusWindow(window);
    g.ActiveIdUsingNavDirMask |= (1 << ImGuiDir_Left) | (1 << ImGuiDir_Right);
  }
  if (clicked) ImGui::SetKeyOwner(ImGuiKey_MouseLeft, id);

  ImRect grab_bb;
  const bool value_changed =
      ImGui::SliderBehavior(frame_bb, id, ImGuiDataType_Float, v, &v_min,
                            &v_max, format, 0, &grab_bb);
  if (value_changed) ImGui::MarkItemEdited(id);

  const float grab_padding = 2.0f;
  const ImRect inner_bb(
      frame_bb.Min.x + grab_padding, frame_bb.Min.y + grab_padding,
      frame_bb.Max.x - grab_padding, frame_bb.Max.y - grab_padding);

  ImU32 frame_col = ImGui::GetColorU32(g.ActiveId == id ? ImGuiCol_FrameBgActive
                                       : hovered ? ImGuiCol_FrameBgHovered
                                                 : ImGuiCol_FrameBg);
  ImGui::RenderNavHighlight(frame_bb, id);
  ImGui::RenderFrame(frame_bb.Min, frame_bb.Max, frame_col, true,
                     style.FrameRounding);

  if (grab_bb.Max.x > grab_bb.Min.x && inner_bb.Max.x > inner_bb.Min.x) {
    if (grab_bb.Min.x > inner_bb.Min.x) {
      ImVec2 fill_min(inner_bb.Min.x, inner_bb.Min.y);
      ImVec2 fill_max(grab_bb.Min.x, inner_bb.Max.y);
      window->DrawList->AddRectFilled(
          fill_min, fill_max, ImGui::ColorConvertFloat4ToU32(fillColor),
          style.FrameRounding, ImDrawFlags_RoundCornersLeft);
    }
    window->DrawList->AddRectFilled(
        grab_bb.Min, grab_bb.Max,
        ImGui::ColorConvertFloat4ToU32(g.ActiveId == id ? grabActiveColor
                                                        : grabColor),
        style.GrabRounding);
  }

  char value_buf[64];
  const char* value_buf_end =
      value_buf + ImGui::DataTypeFormatString(value_buf,
                                              IM_ARRAYSIZE(value_buf),
                                              ImGuiDataType_Float, v, format);
  ImGui::RenderTextClipped(frame_bb.Min, frame_bb.Max, value_buf, value_buf_end,
                           NULL, ImVec2(0.5f, 0.5f));

  return value_changed;
}

PurePursuitRadiusWidget::PurePursuitRadiusWidget() {}

void PurePursuitRadiusWidget::Draw() {
  ImGui::SetNextWindowSize(ImVec2(PP_RAD_WIDGET_WIDTH, 0),
                           ImGuiCond_FirstUseEver);

  if (ImGui::Begin("Pure Pursuit Radius", nullptr,
                   ImGuiWindowFlags_NoCollapse)) {
    ImGuiStyle& style = ImGui::GetStyle();
    float oldFrameRounding = style.FrameRounding;
    float oldFramePaddingY = style.FramePadding.y;
    style.FrameRounding = 6.0f;
    style.FramePadding.y = 8.0f;

    ImGui::PushStyleColor(ImGuiCol_FrameBg, ColorScheme::BG_MEDIUM);
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ColorScheme::BG_LIGHT);
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive, ColorScheme::BG_LIGHTER);

    ImGui::PushStyleColor(ImGuiCol_Text, ColorScheme::PRIMARY_BLUE);
    ImGui::Text("Rad Slow");
    ImGui::PopStyleColor();
    SliderFloatWithFill("##radSlow", &ASTAR_PP_RAD_SLOW, 0.0f, 130.0f, "%.1f",
                        ColorScheme::PRIMARY_BLUE,
                        ColorScheme::Lighten(ColorScheme::PRIMARY_BLUE, 0.2f),
                        ColorScheme::Lighten(ColorScheme::PRIMARY_BLUE, 0.4f));

    ImGui::Spacing();

    ImGui::PushStyleColor(ImGuiCol_Text, ColorScheme::PRIMARY_PURPLE);
    ImGui::Text("Rad Fast");
    ImGui::PopStyleColor();
    SliderFloatWithFill(
        "##radFast", &ASTAR_PP_RAD_FAST, 0.0f, 130.0f, "%.1f",
        ColorScheme::PRIMARY_PURPLE,
        ColorScheme::Lighten(ColorScheme::PRIMARY_PURPLE, 0.2f),
        ColorScheme::Lighten(ColorScheme::PRIMARY_PURPLE, 0.4f));

    ImGui::Spacing();

    ImGui::PushStyleColor(ImGuiCol_Text, ColorScheme::PRIMARY_ORANGE);
    ImGui::Text("Speed Fast");
    ImGui::PopStyleColor();
    SliderFloatWithFill(
        "##speedFast", &ASTAR_PP_SPEED_FAST, 0.0f, 600.0f, "%.0f",
        ColorScheme::PRIMARY_ORANGE,
        ColorScheme::Lighten(ColorScheme::PRIMARY_ORANGE, 0.2f),
        ColorScheme::Lighten(ColorScheme::PRIMARY_ORANGE, 0.4f));

    ImGui::PopStyleColor(3);
    style.FrameRounding = oldFrameRounding;
    style.FramePadding.y = oldFramePaddingY;
  }
  ImGui::End();
}
