#include "ColorScheme.h"
#include <algorithm>
#include <map>

namespace ColorScheme
{
    // ===== UTILITY FUNCTIONS =====
    
    ImVec4 WithAlpha(const ImVec4& color, float alpha)
    {
        return ImVec4(color.x, color.y, color.z, alpha);
    }
    
    ImVec4 Darken(const ImVec4& color, float factor)
    {
        return ImVec4(
            std::max(0.0f, color.x * (1.0f - factor)),
            std::max(0.0f, color.y * (1.0f - factor)),
            std::max(0.0f, color.z * (1.0f - factor)),
            color.w
        );
    }
    
    ImVec4 Lighten(const ImVec4& color, float factor)
    {
        return ImVec4(
            std::min(1.0f, color.x + (1.0f - color.x) * factor),
            std::min(1.0f, color.y + (1.0f - color.y) * factor),
            std::min(1.0f, color.z + (1.0f - color.z) * factor),
            color.w
        );
    }
    
    ImVec4 GetStatusColor(const std::string& status)
    {
        static const std::map<std::string, ImVec4> statusColors = {
            {"success", STATUS_SUCCESS},
            {"warning", STATUS_WARNING},
            {"error", STATUS_ERROR},
            {"info", STATUS_INFO},
            {"ok", STATUS_SUCCESS},
            {"good", STATUS_SUCCESS},
            {"bad", STATUS_ERROR},
            {"critical", STATUS_ERROR}
        };
        
        auto it = statusColors.find(status);
        return (it != statusColors.end()) ? it->second : TEXT_PRIMARY;
    }
    
    ImVec4 GetVariantColor(const std::string& variant)
    {
        static const std::map<std::string, ImVec4> variantColors = {
            {"Camera", PRIMARY_GREEN},
            {"Blob", PRIMARY_BLUE},
            {"Heuristic", PRIMARY_RED},
            {"Neural", PRIMARY_PURPLE},
            {"Fusion", PRIMARY_ORANGE},
            {"NeuralRot", PRIMARY_PURPLE},
            {"Opencv", PRIMARY_BLUE}
        };
        
        auto it = variantColors.find(variant);
        return (it != variantColors.end()) ? it->second : PRIMARY_BLUE;
    }
    
    // ===== COLOR PUSH/POP FUNCTIONS =====
    
    void PushButtonColors(const ImVec4& baseColor)
    {
        ImGui::PushStyleColor(ImGuiCol_Button, baseColor);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, Lighten(baseColor, 0.2f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, Darken(baseColor, 0.2f));
    }
    
    void PopButtonColors()
    {
        ImGui::PopStyleColor(3);
    }
    
    void PushSuccessColors()
    {
        ImGui::PushStyleColor(ImGuiCol_Button, STATUS_SUCCESS);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, Lighten(STATUS_SUCCESS, 0.2f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, Darken(STATUS_SUCCESS, 0.2f));
    }
    
    void PushWarningColors()
    {
        ImGui::PushStyleColor(ImGuiCol_Button, STATUS_WARNING);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, Lighten(STATUS_WARNING, 0.2f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, Darken(STATUS_WARNING, 0.2f));
    }
    
    void PushErrorColors()
    {
        ImGui::PushStyleColor(ImGuiCol_Button, STATUS_ERROR);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, Lighten(STATUS_ERROR, 0.2f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, Darken(STATUS_ERROR, 0.2f));
    }
    
    void PushInfoColors()
    {
        ImGui::PushStyleColor(ImGuiCol_Button, STATUS_INFO);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, Lighten(STATUS_INFO, 0.2f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, Darken(STATUS_INFO, 0.2f));
    }
    
    void PopStatusColors()
    {
        ImGui::PopStyleColor(3);
    }
    
    void PushProgressBarColors(const ImVec4& color)
    {
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, color);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogramHovered, Lighten(color, 0.2f));
    }
    
    void PopProgressBarColors()
    {
        ImGui::PopStyleColor(2);
    }
    
    void PushTableColors()
    {
        ImGui::PushStyleColor(ImGuiCol_TableHeaderBg, BG_LIGHT);
        ImGui::PushStyleColor(ImGuiCol_TableBorderLight, BORDER);
        ImGui::PushStyleColor(ImGuiCol_TableBorderStrong, BORDER);
    }
    
    void PopTableColors()
    {
        ImGui::PopStyleColor(3);
    }
    
    void PushInputColors()
    {
        ImGui::PushStyleColor(ImGuiCol_FrameBg, BG_LIGHT);
        ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, BG_LIGHTER);
        ImGui::PushStyleColor(ImGuiCol_FrameBgActive, BG_LIGHTER);
    }
    
    void PopInputColors()
    {
        ImGui::PopStyleColor(3);
    }
    
    // ===== MAIN COLOR SCHEME APPLICATION =====
    
    void ApplyColorScheme()
    {
        ImGuiStyle& style = ImGui::GetStyle();
        
        // Start with dark theme as base
        ImGui::StyleColorsDark();
        
        // ===== COLORS =====
        ImVec4* colors = style.Colors;
        
        // Window colors
        colors[ImGuiCol_WindowBg] = BG_DARK;
        colors[ImGuiCol_ChildBg] = BG_MEDIUM;
        colors[ImGuiCol_PopupBg] = BG_LIGHT;
        colors[ImGuiCol_TitleBg] = BG_MEDIUM;
        colors[ImGuiCol_TitleBgActive] = BG_LIGHT;
        colors[ImGuiCol_TitleBgCollapsed] = BG_MEDIUM;
        
        // Menu bar colors
        colors[ImGuiCol_MenuBarBg] = BG_MEDIUM;
        
        // Scrollbar colors
        colors[ImGuiCol_ScrollbarBg] = BG_MEDIUM;
        colors[ImGuiCol_ScrollbarGrab] = BG_LIGHTER;
        colors[ImGuiCol_ScrollbarGrabHovered] = Lighten(BG_LIGHTER, 0.2f);
        colors[ImGuiCol_ScrollbarGrabActive] = PRIMARY_BLUE;
        
        // Button colors
        colors[ImGuiCol_Button] = BUTTON_NORMAL;
        colors[ImGuiCol_ButtonHovered] = BUTTON_HOVERED;
        colors[ImGuiCol_ButtonActive] = BUTTON_ACTIVE;
        
        // Input field colors
        colors[ImGuiCol_FrameBg] = BG_LIGHT;
        colors[ImGuiCol_FrameBgHovered] = BG_LIGHTER;
        colors[ImGuiCol_FrameBgActive] = BG_LIGHTER;
        
        // Slider colors
        colors[ImGuiCol_SliderGrab] = SLIDER_GRAB;
        colors[ImGuiCol_SliderGrabActive] = SLIDER_GRAB_ACTIVE;
        
        // Header colors
        colors[ImGuiCol_Header] = BG_LIGHT;
        colors[ImGuiCol_HeaderHovered] = BG_LIGHTER;
        colors[ImGuiCol_HeaderActive] = PRIMARY_BLUE;
        
        // Tab colors
        colors[ImGuiCol_Tab] = BG_MEDIUM;
        colors[ImGuiCol_TabHovered] = BG_LIGHT;
        colors[ImGuiCol_TabActive] = BG_LIGHT;
        colors[ImGuiCol_TabUnfocused] = BG_MEDIUM;
        colors[ImGuiCol_TabUnfocusedActive] = BG_LIGHT;
        
        // Table colors
        colors[ImGuiCol_TableHeaderBg] = BG_LIGHT;
        colors[ImGuiCol_TableBorderLight] = BORDER;
        colors[ImGuiCol_TableBorderStrong] = BORDER;
        colors[ImGuiCol_TableRowBg] = BG_DARK;
        colors[ImGuiCol_TableRowBgAlt] = BG_MEDIUM;
        
        // Separator and border colors
        colors[ImGuiCol_Separator] = SEPARATOR;
        colors[ImGuiCol_SeparatorHovered] = PRIMARY_BLUE;
        colors[ImGuiCol_SeparatorActive] = PRIMARY_BLUE;
        colors[ImGuiCol_Border] = BORDER;
        colors[ImGuiCol_BorderShadow] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
        
        // Docking colors
        colors[ImGuiCol_DockingPreview] = WithAlpha(PRIMARY_BLUE, 0.3f);
        colors[ImGuiCol_DockingEmptyBg] = BG_DARK;
        
        // Plot colors
        colors[ImGuiCol_PlotLines] = PRIMARY_BLUE;
        colors[ImGuiCol_PlotLinesHovered] = Lighten(PRIMARY_BLUE, 0.2f);
        colors[ImGuiCol_PlotHistogram] = PRIMARY_GREEN;
        colors[ImGuiCol_PlotHistogramHovered] = Lighten(PRIMARY_GREEN, 0.2f);
        
        // Text colors
        colors[ImGuiCol_Text] = TEXT_PRIMARY;
        colors[ImGuiCol_TextDisabled] = TEXT_DISABLED;
        
        // Selection colors - using available colors
        colors[ImGuiCol_DragDropTarget] = WithAlpha(PRIMARY_GREEN, 0.3f);
        
        // Navigation colors
        colors[ImGuiCol_NavHighlight] = PRIMARY_BLUE;
        colors[ImGuiCol_NavWindowingHighlight] = WithAlpha(PRIMARY_BLUE, 0.7f);
        colors[ImGuiCol_NavWindowingDimBg] = WithAlpha(BG_DARK, 0.8f);
        
        // Modal colors
        colors[ImGuiCol_ModalWindowDimBg] = WithAlpha(BG_DARK, 0.8f);
        
        // ===== STYLE PROPERTIES =====
        
        // Window styling
        style.WindowRounding = 6.0f;
        style.WindowBorderSize = 1.0f;
        style.WindowPadding = ImVec2(8, 8);
        style.WindowMinSize = ImVec2(100, 100);
        
        // Child window styling
        style.ChildRounding = 4.0f;
        style.ChildBorderSize = 1.0f;
        
        // Popup styling
        style.PopupRounding = 4.0f;
        style.PopupBorderSize = 1.0f;
        
        // Frame styling
        style.FrameRounding = 4.0f;
        style.FrameBorderSize = 0.0f;
        style.FramePadding = ImVec2(4, 3);
        
        // Item styling
        style.ItemSpacing = ImVec2(8, 4);
        style.ItemInnerSpacing = ImVec2(4, 4);
        
        // Slider styling
        style.GrabRounding = 4.0f;
        style.GrabMinSize = 10.0f;
        
        // Tab styling
        style.TabRounding = 4.0f;
        style.TabBorderSize = 0.0f;
        style.TabMinWidthForCloseButton = 0.0f;
        
        // Table styling
        // style.TableRounding = 4.0f;
        // style.TableBorderSize = 1.0f;
        
        // Scrollbar styling
        style.ScrollbarRounding = 4.0f;
        style.ScrollbarSize = 12.0f;
        
        // Docking styling
        style.DockingSeparatorSize = 2.0f;
        
        // Alpha and anti-aliasing
        style.Alpha = 1.0f;
        style.DisabledAlpha = 0.6f;
        style.AntiAliasedLines = true;
        style.AntiAliasedLinesUseTex = true;
        style.AntiAliasedFill = true;
    }
}
