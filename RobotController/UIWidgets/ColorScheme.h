#pragma once

#include "imgui.h"
#include <string>

namespace ColorScheme
{
    // ===== CORE COLOR PALETTE =====
    // Primary colors - darker and more muted
    const ImVec4 PRIMARY_BLUE = ImVec4(0.15f, 0.45f, 0.8f, 1.0f);      // Darker blue for primary actions
    const ImVec4 PRIMARY_GREEN = ImVec4(0.15f, 0.6f, 0.3f, 1.0f);      // Darker green for success/positive states
    const ImVec4 PRIMARY_RED = ImVec4(0.7f, 0.2f, 0.2f, 1.0f);         // Darker red for errors/danger
    const ImVec4 PRIMARY_ORANGE = ImVec4(0.8f, 0.45f, 0.15f, 1.0f);    // Darker orange for warnings
    const ImVec4 PRIMARY_PURPLE = ImVec4(0.55f, 0.3f, 0.8f, 1.0f);     // Darker purple for special elements
    
    // Background colors - much darker
    const ImVec4 BG_DARK = ImVec4(0.03f, 0.03f, 0.05f, 1.0f);          // Very dark main background
    const ImVec4 BG_MEDIUM = ImVec4(0.08f, 0.08f, 0.1f, 1.0f);         // Darker secondary background
    const ImVec4 BG_LIGHT = ImVec4(0.12f, 0.12f, 0.15f, 1.0f);         // Darker elevated elements
    const ImVec4 BG_LIGHTER = ImVec4(0.16f, 0.16f, 0.19f, 1.0f);       // Darker hover states
    
    // Text colors - slightly darker
    const ImVec4 TEXT_PRIMARY = ImVec4(0.9f, 0.9f, 0.9f, 1.0f);        // Slightly darker primary text
    const ImVec4 TEXT_SECONDARY = ImVec4(0.6f, 0.6f, 0.6f, 1.0f);      // Darker secondary text
    const ImVec4 TEXT_DISABLED = ImVec4(0.4f, 0.4f, 0.4f, 1.0f);       // Darker disabled text
    const ImVec4 TEXT_ACCENT = ImVec4(0.25f, 0.55f, 0.9f, 1.0f);       // Darker accent text
    
    // Border and separator colors - darker
    const ImVec4 BORDER = ImVec4(0.18f, 0.18f, 0.21f, 1.0f);           // Darker borders
    const ImVec4 SEPARATOR = ImVec4(0.22f, 0.22f, 0.25f, 1.0f);        // Darker separators
    
    // Interactive element colors - darker
    const ImVec4 BUTTON_NORMAL = ImVec4(0.1f, 0.1f, 0.13f, 1.0f);      // Darker button normal state
    const ImVec4 BUTTON_HOVERED = ImVec4(0.14f, 0.14f, 0.18f, 1.0f);   // Darker button hovered
    const ImVec4 BUTTON_ACTIVE = ImVec4(0.18f, 0.18f, 0.22f, 1.0f);    // Darker button active/pressed
    
    const ImVec4 SLIDER_GRAB = ImVec4(0.2f, 0.45f, 0.8f, 1.0f);        // Darker slider grab handle
    const ImVec4 SLIDER_GRAB_ACTIVE = ImVec4(0.3f, 0.55f, 0.9f, 1.0f); // Darker slider grab active
    
    // Status colors - darker and more muted
    const ImVec4 STATUS_SUCCESS = ImVec4(0.15f, 0.55f, 0.25f, 1.0f);   // Darker success status
    const ImVec4 STATUS_WARNING = ImVec4(0.7f, 0.45f, 0.15f, 1.0f);    // Darker warning status
    const ImVec4 STATUS_ERROR = ImVec4(0.7f, 0.2f, 0.2f, 1.0f);        // Darker error status
    const ImVec4 STATUS_INFO = ImVec4(0.2f, 0.45f, 0.8f, 1.0f);        // Darker info status
    
    // ===== UTILITY FUNCTIONS =====
    
    // Apply the complete color scheme to ImGui
    void ApplyColorScheme();
    
    // Get a color with custom alpha
    ImVec4 WithAlpha(const ImVec4& color, float alpha);
    
    // Get a darker/lighter version of a color
    ImVec4 Darken(const ImVec4& color, float factor = 0.3f);
    ImVec4 Lighten(const ImVec4& color, float factor = 0.3f);
    
    // Get semantic colors for different states
    ImVec4 GetStatusColor(const std::string& status);
    ImVec4 GetVariantColor(const std::string& variant);
    
    // Push/pop color pairs for common UI patterns
    void PushButtonColors(const ImVec4& baseColor);
    void PopButtonColors();
    
    void PushSuccessColors();
    void PushWarningColors();
    void PushErrorColors();
    void PushInfoColors();
    void PopStatusColors();
    
    // ===== WIDGET-SPECIFIC COLOR HELPERS =====
    
    // Progress bar colors
    void PushProgressBarColors(const ImVec4& color);
    void PopProgressBarColors();
    
    // Table colors
    void PushTableColors();
    void PopTableColors();
    
    // Input field colors
    void PushInputColors();
    void PopInputColors();
}
