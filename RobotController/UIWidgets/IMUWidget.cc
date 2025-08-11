#include "IMUWidget.h"
#include "../RobotController.h"
#include "../GuiUtils.h"
#include "imgui.h"
#include "../SafeDrawing.h"
#include "ColorScheme.h"
#include <cmath>
#include <algorithm>

#define REFRESH_INTERVAL_MS 30
#define WIDGET_WIDTH 400
#define WIDGET_HEIGHT 500

// Helper function to wrap angle to [-π, π]
float WrapAngle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

IMUWidget::IMUWidget() : ImageWidget("IMU", false)
{
    AddAdditionalUI([](){
        // button
        if (ImGui::Button("Flip angle"))
        {
            RobotOdometry& odometry = RobotController::GetInstance().odometry;
            odometry.UpdateForceSetAngle(odometry.Robot().GetAngle() + M_PI, false);
        }
    });
}



void DrawAccelerationVector(const ImVec2& center, float accelX, float accelY, float maxAccel, float radius)
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    
    // Normalize acceleration
    float normalizedX = accelX / maxAccel;
    float normalizedY = accelY / maxAccel;
    float magnitude = std::sqrt(normalizedX * normalizedX + normalizedY * normalizedY);
    
    // Clamp to circle
    if (magnitude > 1.0f) {
        normalizedX /= magnitude;
        normalizedY /= magnitude;
        magnitude = 1.0f;
    }
    
    // Draw background circle
    draw_list->AddCircle(center, radius, ImGui::ColorConvertFloat4ToU32(ColorScheme::BG_MEDIUM), 0, 2.0f);
    
    // Draw grid lines
    for (int i = 1; i <= 4; i++) {
        float gridRadius = (radius * i) / 4.0f;
        draw_list->AddCircle(center, gridRadius, ImGui::ColorConvertFloat4ToU32(ColorScheme::BORDER), 0, 1.0f);
    }
    
    // Draw crosshair
    draw_list->AddLine(
        ImVec2(center.x - radius, center.y),
        ImVec2(center.x + radius, center.y),
        ImGui::ColorConvertFloat4ToU32(ColorScheme::BORDER), 1.0f
    );
    draw_list->AddLine(
        ImVec2(center.x, center.y - radius),
        ImVec2(center.x, center.y + radius),
        ImGui::ColorConvertFloat4ToU32(ColorScheme::BORDER), 1.0f
    );
    
    // Draw acceleration vector
    if (magnitude > 0.01f) {
        ImVec2 vectorEnd(
            center.x + normalizedX * radius * 0.8f,
            center.y + normalizedY * radius * 0.8f
        );
        
        // Vector line
        draw_list->AddLine(center, vectorEnd, 
                          ImGui::ColorConvertFloat4ToU32(ColorScheme::PRIMARY_BLUE), 3.0f);
        
        // Vector arrow head
        float arrowLength = 12.0f;
        float arrowAngle = std::atan2(normalizedY, normalizedX);
        
        ImVec2 arrowTip1(
            vectorEnd.x - arrowLength * std::cos(arrowAngle - M_PI / 6.0f),
            vectorEnd.y - arrowLength * std::sin(arrowAngle - M_PI / 6.0f)
        );
        ImVec2 arrowTip2(
            vectorEnd.x - arrowLength * std::cos(arrowAngle + M_PI / 6.0f),
            vectorEnd.y - arrowLength * std::sin(arrowAngle + M_PI / 6.0f)
        );
        
        draw_list->AddLine(vectorEnd, arrowTip1, 
                          ImGui::ColorConvertFloat4ToU32(ColorScheme::PRIMARY_BLUE), 2.0f);
        draw_list->AddLine(vectorEnd, arrowTip2, 
                          ImGui::ColorConvertFloat4ToU32(ColorScheme::PRIMARY_BLUE), 2.0f);
        
        // Magnitude indicator (colored circle)
        ImVec4 magnitudeColor = ColorScheme::GetInterpolatedColor(magnitude);
        draw_list->AddCircleFilled(vectorEnd, 6.0f, 
                                  ImGui::ColorConvertFloat4ToU32(magnitudeColor));
    }
    
    // Draw coordinate labels
    draw_list->AddText(ImVec2(center.x + radius + 5, center.y - 8), 
                       ImGui::ColorConvertFloat4ToU32(ColorScheme::TEXT_SECONDARY), "X");
    draw_list->AddText(ImVec2(center.x - 8, center.y - radius - 20), 
                       ImGui::ColorConvertFloat4ToU32(ColorScheme::TEXT_SECONDARY), "Y");
}

void DrawRotationIndicator(const ImVec2& center, float rotation, float radius, const ImVec4& color)
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    
    // Draw outer ring
    draw_list->AddCircle(center, radius, ImGui::ColorConvertFloat4ToU32(ColorScheme::BORDER), 0, 3.0f);
    
    // Draw rotation indicator with angle wrapping
    float wrappedRotation = WrapAngle(rotation);
    float startAngle = -M_PI / 2.0f; // Start from top
    float endAngle = startAngle + wrappedRotation;
    
    // Draw arc using multiple line segments
    const int segments = 64; // More segments for smoother arc
    for (int i = 0; i < segments; i++) {
        float angle1 = startAngle + (endAngle - startAngle) * i / (float)segments;
        float angle2 = startAngle + (endAngle - startAngle) * (i + 1) / (float)segments;
        
        ImVec2 point1(center.x + (radius - 8) * cos(angle1), center.y + (radius - 8) * sin(angle1));
        ImVec2 point2(center.x + (radius - 8) * cos(angle2), center.y + (radius - 8) * sin(angle2));
        
        draw_list->AddLine(point1, point2, ImGui::ColorConvertFloat4ToU32(color), 6.0f);
    }
    
    // Draw arrow pointing in rotation direction
    float arrowLength = 20.0f;
    ImVec2 arrowTip(
        center.x + (radius - 15) * std::cos(endAngle),
        center.y + (radius - 15) * std::sin(endAngle)
    );
    
    // Arrow head
    float arrowAngle = endAngle;
    ImVec2 arrowBase1(
        arrowTip.x - arrowLength * std::cos(arrowAngle - M_PI / 6.0f),
        arrowTip.y - arrowLength * std::sin(arrowAngle - M_PI / 6.0f)
    );
    ImVec2 arrowBase2(
        arrowTip.x - arrowLength * std::cos(arrowAngle + M_PI / 6.0f),
        arrowTip.y - arrowLength * std::sin(arrowAngle + M_PI / 6.0f)
    );
    
    draw_list->AddTriangleFilled(arrowTip, arrowBase1, arrowBase2, 
                                 ImGui::ColorConvertFloat4ToU32(color));
    
    // Draw rotation value
    char rotText[32];
    snprintf(rotText, sizeof(rotText), "%.1f°", wrappedRotation * 180.0f / M_PI);
    
    ImVec2 textSize = ImGui::CalcTextSize(rotText);
    ImVec2 textPos(center.x - textSize.x * 0.5f, center.y + radius + 20);
    draw_list->AddText(textPos, ImGui::ColorConvertFloat4ToU32(ColorScheme::TEXT_PRIMARY), rotText);
}

void DrawVelocityVector(const ImVec2& center, float velX, float velY, float maxVel, float radius)
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    
    // Normalize velocity
    float normalizedX = velX / maxVel;
    float normalizedY = velY / maxVel;
    float magnitude = std::sqrt(normalizedX * normalizedX + normalizedY * normalizedY);
    
    // Clamp to circle
    if (magnitude > 1.0f) {
        normalizedX /= magnitude;
        normalizedY /= magnitude;
        magnitude = 1.0f;
    }
    
    // Draw velocity vector only if there's significant movement
    if (magnitude > 0.01f) {
        ImVec2 vectorEnd(
            center.x + normalizedX * radius * 0.7f,
            center.y + normalizedY * radius * 0.7f
        );
        
        // Vector line with gradient effect
        const int segments = 16;
        for (int i = 0; i < segments; i++) {
            float t = i / (float)segments;
            ImVec2 start(center.x + normalizedX * radius * 0.7f * t, center.y + normalizedY * radius * 0.7f * t);
            ImVec2 end(center.x + normalizedX * radius * 0.7f * (t + 1.0f/segments), center.y + normalizedY * radius * 0.7f * (t + 1.0f/segments));
            
            // Gradient from center to tip
            ImVec4 lineColor = ColorScheme::GetInterpolatedColor(t);
            draw_list->AddLine(start, end, ImGui::ColorConvertFloat4ToU32(lineColor), 4.0f);
        }
        
        // Vector arrow head
        float arrowLength = 16.0f;
        float arrowAngle = std::atan2(normalizedY, normalizedX);
        
        ImVec2 arrowTip1(
            vectorEnd.x - arrowLength * std::cos(arrowAngle - M_PI / 6.0f),
            vectorEnd.y - arrowLength * std::sin(arrowAngle - M_PI / 6.0f)
        );
        ImVec2 arrowTip2(
            vectorEnd.x - arrowLength * std::cos(arrowAngle + M_PI / 6.0f),
            vectorEnd.y - arrowLength * std::sin(arrowAngle + M_PI / 6.0f)
        );
        
        draw_list->AddLine(vectorEnd, arrowTip1, 
                          ImGui::ColorConvertFloat4ToU32(ColorScheme::PRIMARY_BLUE), 3.0f);
        draw_list->AddLine(vectorEnd, arrowTip2, 
                          ImGui::ColorConvertFloat4ToU32(ColorScheme::PRIMARY_BLUE), 3.0f);
        
        // Magnitude indicator (colored circle with pulsing effect)
        ImVec4 magnitudeColor = ColorScheme::GetInterpolatedColor(magnitude);
        float pulseRadius = 8.0f + 2.0f * sin(ImGui::GetTime() * 4.0f); // Pulsing effect
        draw_list->AddCircleFilled(vectorEnd, pulseRadius, 
                                  ImGui::ColorConvertFloat4ToU32(magnitudeColor));
    }
    
    // Draw coordinate labels
    draw_list->AddText(ImVec2(center.x + radius + 8, center.y - 10), 
                       ImGui::ColorConvertFloat4ToU32(ColorScheme::TEXT_SECONDARY), "X");
    draw_list->AddText(ImVec2(center.x - 10, center.y - radius - 25), 
                       ImGui::ColorConvertFloat4ToU32(ColorScheme::TEXT_SECONDARY), "Y");
}

void IMUWidget::Draw()
{
    // Get the latest IMU data and robot data
    IMUData imuData = RobotController::GetInstance().GetIMUData();
    RobotMessage imuDebugMessage = RobotController::GetInstance().GetRobotLink().GetLastIMUDebugMessage();
    RobotOdometry& odometry = RobotController::GetInstance().odometry;
    OdometryData robot = odometry.Robot();
    
    // Set window size
    ImGui::SetNextWindowSize(ImVec2(WIDGET_WIDTH, WIDGET_HEIGHT), ImGuiCond_FirstUseEver);
    
    if (ImGui::Begin("IMU Data", nullptr, ImGuiWindowFlags_NoCollapse))
    {
        ImVec2 windowPos = ImGui::GetWindowPos();
        ImVec2 windowSize = ImGui::GetWindowSize();
        ImVec2 contentStart = ImGui::GetCursorScreenPos();
        
        // Header with status indicator
        ImGui::PushStyleColor(ImGuiCol_Text, ColorScheme::TEXT_PRIMARY);
        ImGui::Text("IMU Sensor Data");
        ImGui::PopStyleColor();
        
        ImGui::SameLine();
        
        // Health status indicator
        bool isHealthy = false;
        bool hasDebugData = false;
        if (imuDebugMessage.type == RobotMessageType::IMU_DEBUG_DATA) {
            isHealthy = imuDebugMessage.imuDebugData.isHealthy;
            hasDebugData = true;
        }
        
        ImVec4 statusColor = hasDebugData ? (isHealthy ? ColorScheme::STATUS_SUCCESS : ColorScheme::STATUS_ERROR) : ColorScheme::STATUS_WARNING;
        ImGui::PushStyleColor(ImGuiCol_Text, statusColor);
        if (hasDebugData) {
            ImGui::Text("● %s", isHealthy ? "HEALTHY" : "UNHEALTHY");
        } else {
            ImGui::Text("● NO DATA");
        }
        ImGui::PopStyleColor();
        
        ImGui::Separator();
        
        // Main rotation display that fills the width
        ImGui::Text("Rotation & Velocity");
        ImGui::Spacing();
        
        // Calculate the available width for the rotation display
        float availableWidth = windowSize.x - 40; // Leave some margin
        float rotationRadius = availableWidth * 0.4f; // Use 40% of available width for rotation
        
        // Center the rotation display
        ImVec2 rotationCenter(contentStart.x + availableWidth * 0.5f, contentStart.y + 160);
        
        double angle = robot.GetAngle() + M_PI / 2;
        // Draw the main rotation indicator
        DrawRotationIndicator(rotationCenter, angle, rotationRadius, ColorScheme::PRIMARY_PURPLE);
        
        // Draw velocity vector inside the rotation display
        float velX = robot.robotVelocity.x / 10.0f;
        float velY = robot.robotVelocity.y / 10.0f;
        float maxVel = 5.0f; // Max velocity for normalization
        
        DrawVelocityVector(rotationCenter, velX, velY, maxVel, rotationRadius * 0.8f);
        
        // Velocity values display
        ImGui::SetCursorScreenPos(ImVec2(contentStart.x + 20, contentStart.y + 320));
        ImGui::BeginGroup();
        ImGui::PushStyleColor(ImGuiCol_Text, ColorScheme::TEXT_SECONDARY);
        ImGui::Text("Velocity X: %.2f m/s", velX);
        ImGui::Text("Velocity Y: %.2f m/s", velY);
        ImGui::Text("Velocity Magnitude: %.2f m/s", std::sqrt(velX * velX + velY * velY));
        ImGui::PopStyleColor();
        ImGui::EndGroup();
        
        // CV Rotation if available
        CVRotation* pcvRotation = CVRotation::GetInstance();
        if (pcvRotation != nullptr && pcvRotation->IsRunning()) {
            double cvRotation = pcvRotation->GetLastComputedRotation();
            float confidence = pcvRotation->GetLastConfidence();
            
            ImGui::SameLine();
            ImGui::SetCursorScreenPos(ImVec2(contentStart.x + availableWidth * 0.7f, contentStart.y + 290));
            
            ImGui::BeginGroup();
            ImGui::PushStyleColor(ImGuiCol_Text, ColorScheme::TEXT_SECONDARY);
            ImGui::Text("CV Rotation: %.1f°", WrapAngle(cvRotation) * 180.0f / M_PI);
            ImGui::Text("Confidence: %.1f%%", confidence * 100.0f);
            ImGui::PopStyleColor();
            ImGui::EndGroup();
        }
        
        // Add spacing to move debug section down
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::Spacing();
        
        // IMU Debug Data
        if (imuDebugMessage.type == RobotMessageType::IMU_DEBUG_DATA) {
            ImGui::Spacing();
            ImGui::Separator();
            
            ImGui::Text("Debug Information");
            ImGui::Spacing();
            
            // Create a table for debug data
            if (ImGui::BeginTable("IMUDebug", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("Metric", ImGuiTableColumnFlags_WidthFixed, 120.0f);
                ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
                
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Rotation");
                ImGui::TableNextColumn();
                ImGui::Text("%.1f°", WrapAngle(imuDebugMessage.imuDebugData.myCurrRotationRad) * 180.0f / M_PI);
                
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Velocity Diff");
                ImGui::TableNextColumn();
                ImGui::Text("%.3f rad/s", imuDebugMessage.imuDebugData.avgVelocityDifference);
                
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("External Vel");
                ImGui::TableNextColumn();
                ImGui::Text("%.3f rad/s", imuDebugMessage.imuDebugData.externalMeanVelocity);
                
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Error");
                ImGui::TableNextColumn();
                
                float errorPercent = imuDebugMessage.imuDebugData.percentageError;
                ImVec4 errorColor = ColorScheme::GetInterpolatedColor(1.0f - errorPercent / 100.0f);
                ImGui::PushStyleColor(ImGuiCol_Text, errorColor);
                ImGui::Text("%.1f%%", errorPercent);
                ImGui::PopStyleColor();
                
                ImGui::EndTable();
            }
        }
        
        // Flip angle button
        ImGui::Spacing();
        ImGui::Separator();
        
        if (ImGui::Button("Flip Angle (180°)", ImVec2(-1, 0))) {
            RobotOdometry& odometry = RobotController::GetInstance().odometry;
            odometry.UpdateForceSetAngle(odometry.Robot().GetAngle() + M_PI, false);
        }
    }
    
    ImGui::End();
}