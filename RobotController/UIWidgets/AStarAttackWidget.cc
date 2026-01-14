#include "AStarAttackWidget.h"
#include "../RobotController.h"
#include "../Strategies/AStarAttack.h"
#include "../SafeDrawing.h"
#include "../RobotConfig.h"
#include "imgui.h"
#include <algorithm>
#include <limits>

AStarAttackWidget* AStarAttackWidget::_instance = nullptr;

AStarAttackWidget::AStarAttackWidget() : ImageWidget("Orbit", false)
{
    _instance = this;
    
    // Initialize with GUI controls
    AddAdditionalUI([this](){
        DrawGUI();
    });
    
    // Load initial values from AStarAttack if available
    AStarAttack* astar = AStarAttack::GetInstance();
    if (astar) {
        astar->GetRadiusCurvePoints(_radiusCurveX, _radiusCurveY);
    }
}

AStarAttackWidget* AStarAttackWidget::GetInstance()
{
    return _instance;
}

void AStarAttackWidget::Draw()
{
    // Sync parameters occasionally (not every frame for performance)
    static int frameCounter = 0;
    if (++frameCounter % 30 == 0) { // Sync every 30 frames
        SyncFromAStarAttack();
    }
    
    _DrawCurveVisualization();
    ImageWidget::Draw();
}

void AStarAttackWidget::_DrawCurveVisualization()
{
    // Create the visualization canvas
    cv::Mat canvas = cv::Mat::zeros(CURVE_WIDGET_HEIGHT, CURVE_WIDGET_WIDTH, CV_8UC3);
    
    // Background color (dark gray)
    canvas.setTo(cv::Scalar(40, 40, 40));
    
    // Draw grid if enabled
    if (_showGrid) {
        cv::Scalar gridColor(80, 80, 80);
        
        // Vertical grid lines
        for (int x = 0; x < CURVE_WIDGET_WIDTH; x += 30) {
            cv::line(canvas, cv::Point(x, 0), cv::Point(x, CURVE_WIDGET_HEIGHT), gridColor, 1);
        }
        
        // Horizontal grid lines
        for (int y = 0; y < CURVE_WIDGET_HEIGHT; y += 25) {
            cv::line(canvas, cv::Point(0, y), cv::Point(CURVE_WIDGET_WIDTH, y), gridColor, 1);
        }
    }
    
    // Draw axes
    cv::Scalar axisColor(200, 200, 200);
    
    // X-axis (bottom)
    cv::line(canvas, cv::Point(0, CURVE_WIDGET_HEIGHT - 1), 
             cv::Point(CURVE_WIDGET_WIDTH, CURVE_WIDGET_HEIGHT - 1), axisColor, 2);
    
    // Y-axis (left)
    cv::line(canvas, cv::Point(0, 0), cv::Point(0, CURVE_WIDGET_HEIGHT), axisColor, 2);
    
    // Convert curve points to screen coordinates
    std::vector<cv::Point> screenPoints;
    for (int i = 0; i < 3; i++) {
        int screenX = (_radiusCurveX[i] / _maxX) * CURVE_WIDGET_WIDTH;
        int screenY = CURVE_WIDGET_HEIGHT - ((_radiusCurveY[i] / _maxY) * CURVE_WIDGET_HEIGHT);
        screenX = std::clamp(screenX, 0, CURVE_WIDGET_WIDTH - 1);
        screenY = std::clamp(screenY, 0, CURVE_WIDGET_HEIGHT - 1);
        screenPoints.push_back(cv::Point(screenX, screenY));
    }
    
    // Draw the piecewise linear curve
    cv::Scalar curveColor(0, 255, 0); // Green
    for (int i = 0; i < screenPoints.size() - 1; i++) {
        cv::line(canvas, screenPoints[i], screenPoints[i + 1], curveColor, 3);
    }
    
    // Draw control points
    cv::Scalar pointColor(255, 255, 0); // Yellow
    for (int i = 0; i < screenPoints.size(); i++) {
        safe_circle(canvas, screenPoints[i], 6, pointColor, -1);
        
        // Add point labels
        std::string label = "P" + std::to_string(i);
        cv::putText(canvas, label, 
                   cv::Point(screenPoints[i].x + 10, screenPoints[i].y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, pointColor, 1);
    }
    
    // Add axis labels
    cv::Scalar labelColor(255, 255, 255);
    cv::putText(canvas, "Fraction", cv::Point(CURVE_WIDGET_WIDTH - 60, CURVE_WIDGET_HEIGHT - 10),
               cv::FONT_HERSHEY_SIMPLEX, 0.4, labelColor, 1);
    cv::putText(canvas, "Radius", cv::Point(5, 15),
               cv::FONT_HERSHEY_SIMPLEX, 0.4, labelColor, 1);
    
    // Add scale markers
    for (int i = 0; i <= 4; i++) {
        float xVal = (i / 4.0f) * _maxX;
        int xPos = (i / 4.0f) * CURVE_WIDGET_WIDTH;
        std::string xLabel = std::to_string((int)xVal);
        cv::putText(canvas, xLabel, cv::Point(xPos - 10, CURVE_WIDGET_HEIGHT - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.3, labelColor, 1);
    }
    
    for (int i = 0; i <= 4; i++) {
        float yVal = (i / 4.0f) * _maxY;
        int yPos = CURVE_WIDGET_HEIGHT - ((i / 4.0f) * CURVE_WIDGET_HEIGHT);
        std::string yLabel = std::to_string((int)yVal);
        cv::putText(canvas, yLabel, cv::Point(2, yPos + 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.3, labelColor, 1);
    }
    
    UpdateMat(canvas);
}

void AStarAttackWidget::DrawGUI()
{
    ImGui::Text("Orbit Parameters");
    ImGui::Separator();
    
    // Radius curve section
    if (ImGui::CollapsingHeader("Radius Curve", ImGuiTreeNodeFlags_DefaultOpen)) {
        _DrawRadiusCurveSliders();
        
        // Control buttons
        if (ImGui::Button("Reset Curve to Default")) {
            _radiusCurveX[0] = 0.0f;   _radiusCurveY[0] = 0.0f;
            _radiusCurveX[1] = 15.0f;  _radiusCurveY[1] = 85.0f;
            _radiusCurveX[2] = 100.0f; _radiusCurveY[2] = 150.0f;
            _UpdateAStarParameters();
        }
        
        ImGui::SameLine();
        ImGui::Checkbox("Show Grid", &_showGrid);
        
        // Display current curve equation info
        ImGui::Text("Input: ETA fraction (0.0 = good, high = bad)");
        ImGui::Text("Output: Attack radius (pixels)");
        
        // Show current values
        ImGui::Text("Current Points:");
        for (int i = 0; i < 3; i++) {
            ImGui::Text("P%d: (%.1f, %.1f)", i, _radiusCurveX[i], _radiusCurveY[i]);
        }
    }
    
    ImGui::Separator();
    
    // Pure Pursuit section
    if (ImGui::CollapsingHeader("Pure Pursuit", ImGuiTreeNodeFlags_DefaultOpen)) {
        _DrawPurePursuitSliders();
    }
    
    ImGui::Separator();
    
    // Direction Scores Breakdown section
    if (ImGui::CollapsingHeader("Direction Scores Breakdown")) {
        _DrawDirectionScoresBreakdown();
    }
}

void AStarAttackWidget::_DrawRadiusCurveSliders()
{
    bool changed = false;
    
    ImGui::Text("X Values (Fraction):");
    ImGui::PushItemWidth(200);
    
    // X values
    if (ImGui::SliderFloat("X0##radiusCurveX0", &_radiusCurveX[0], 0.0f, 50.0f, "%.1f")) changed = true;
    if (ImGui::SliderFloat("X1##radiusCurveX1", &_radiusCurveX[1], 0.0f, 50.0f, "%.1f")) changed = true;
    if (ImGui::SliderFloat("X2##radiusCurveX2", &_radiusCurveX[2], 0.0f, 200.0f, "%.1f")) changed = true;
    
    ImGui::Text("Y Values (Radius):");
    
    // Y values
    if (ImGui::SliderFloat("Y0##radiusCurveY0", &_radiusCurveY[0], 0.0f, 300.0f, "%.1f")) changed = true;
    if (ImGui::SliderFloat("Y1##radiusCurveY1", &_radiusCurveY[1], 0.0f, 300.0f, "%.1f")) changed = true;
    if (ImGui::SliderFloat("Y2##radiusCurveY2", &_radiusCurveY[2], 0.0f, 300.0f, "%.1f")) changed = true;
    
    ImGui::PopItemWidth();
    
    // Ensure X values are ordered
    if (changed) {
        // Sort X values while maintaining Y correspondence
        for (int i = 0; i < 2; i++) {
            for (int j = i + 1; j < 3; j++) {
                if (_radiusCurveX[i] > _radiusCurveX[j]) {
                    std::swap(_radiusCurveX[i], _radiusCurveX[j]);
                    std::swap(_radiusCurveY[i], _radiusCurveY[j]);
                }
            }
        }
        
        _UpdateAStarParameters();
    }
}

void AStarAttackWidget::_DrawPurePursuitSliders()
{
    ImGui::Text("Pure Pursuit Parameters:");
    ImGui::PushItemWidth(200);
    
    // Slow speed radius
    if (ImGui::SliderFloat("Slow Speed Radius##ppRadSlow", &ASTAR_PP_RAD_SLOW, 10.0f, 200.0f, "%.1f pixels")) {
        // Value automatically updates since it's a reference to the global variable
    }
    
    // Fast speed radius
    if (ImGui::SliderFloat("Fast Speed Radius##ppRadFast", &ASTAR_PP_RAD_FAST, 10.0f, 300.0f, "%.1f pixels")) {
        // Value automatically updates since it's a reference to the global variable
    }
    
    // Speed threshold for fast radius
    if (ImGui::SliderFloat("Fast Speed Threshold##ppSpeedFast", &ASTAR_PP_SPEED_FAST, 100.0f, 800.0f, "%.1f px/s")) {
        // Value automatically updates since it's a reference to the global variable
    }
    
    ImGui::PopItemWidth();
    
    // Help text
    ImGui::TextWrapped("Slow Speed Radius: Pure pursuit radius at low speeds");
    ImGui::TextWrapped("Fast Speed Radius: Pure pursuit radius at high speeds");
    ImGui::TextWrapped("Fast Speed Threshold: Speed at which fast radius is reached");
}

void AStarAttackWidget::_UpdateAStarParameters()
{
    // Update the AStarAttack parameters
    AStarAttack* astar = AStarAttack::GetInstance();
    if (astar) {
        astar->SetRadiusCurvePoints(_radiusCurveX, _radiusCurveY);
    }
}

void AStarAttackWidget::SyncFromAStarAttack()
{
    // Sync parameters from AStarAttack (in case they were changed elsewhere)
    AStarAttack* astar = AStarAttack::GetInstance();
    if (astar) {
        astar->GetRadiusCurvePoints(_radiusCurveX, _radiusCurveY);
    }
}

void AStarAttackWidget::_DrawDirectionScoresBreakdown()
{
    AStarAttack* astar = AStarAttack::GetInstance();
    if (!astar) {
        ImGui::Text("AStarAttack instance not available");
        return;
    }
    
    const std::vector<FollowPoint>& followPoints = astar->GetFollowPoints();
    
    if (followPoints.empty()) {
        ImGui::Text("No follow points available (waiting for Execute() to run)");
        return;
    }
    
    // Score names (matching the order in directionScore function)
    const char* scoreNames[] = {
        "Radius",
        "Go Around Angle",
        "Turn Score",
        "Turn Past Opp",
        "Wall Score",
        "Momentum",
        "Direction Agreement",
        "Total Score"
    };
    
    // Find the best (lowest total score) follow point index
    int bestIndex = -1;
    float bestScore = (std::numeric_limits<float>::max)();
    for (size_t i = 0; i < followPoints.size(); i++) {
        const std::vector<float>& scores = followPoints[i].directionScores;
        if (!scores.empty()) {
            float totalScore = scores.back(); // last entry is total score
            if (totalScore < bestScore) {
                bestScore = totalScore;
                bestIndex = (int)i;
            }
        }
    }
    
    // Create a table to display the breakdown
    if (ImGui::BeginTable("DirectionScoresTable", (int)followPoints.size() + 1, 
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollX)) {
        
        // Header row with follow point labels
        ImGui::TableSetupColumn("Score Component", ImGuiTableColumnFlags_WidthFixed, 150.0f);
        for (size_t i = 0; i < followPoints.size(); i++) {
            std::string label = "FP" + std::to_string(i);
            if (followPoints[i].CW) label += " CW";
            else label += " CCW";
            if (followPoints[i].forward) label += " Fwd";
            else label += " Bwd";
            
            ImGui::TableSetupColumn(label.c_str(), ImGuiTableColumnFlags_WidthFixed, 100.0f);
        }
        ImGui::TableHeadersRow();
        
        // Data rows - one for each score component
        for (int scoreIdx = 0; scoreIdx < 8; scoreIdx++) {
            ImGui::TableNextRow();
            
            // First column: score name
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%s", scoreNames[scoreIdx]);
            
            // Remaining columns: score values for each follow point
            for (size_t fpIdx = 0; fpIdx < followPoints.size(); fpIdx++) {
                ImGui::TableSetColumnIndex((int)fpIdx + 1);
                
                const std::vector<float>& scores = followPoints[fpIdx].directionScores;
                
                if (scoreIdx < (int)scores.size()) {
                    float value = scores[scoreIdx];
                    ImGui::Text("%.1f", value);
                    
                    // Highlight the chosen column (same green as total score row)
                    bool isBestColumn = (bestIndex >= 0 && (int)fpIdx == bestIndex);
                    
                    // Highlight the total score row with green
                    if (scoreIdx == 7) {
                        if (isBestColumn) {
                            // Brighter green for the chosen column's total score
                            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, 
                                                 ImGui::GetColorU32(ImVec4(0.5f, 0.7f, 0.5f, 0.5f)));
                        } else {
                            // Normal green for other total scores
                            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, 
                                                 ImGui::GetColorU32(ImVec4(0.3f, 0.5f, 0.3f, 0.3f)));
                        }
                    } else if (isBestColumn) {
                        // Highlight the entire chosen column with the same green as total score row
                        ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, 
                                             ImGui::GetColorU32(ImVec4(0.3f, 0.5f, 0.3f, 0.3f)));
                    }
                } else {
                    ImGui::Text("-");
                }
            }
        }
        
        ImGui::EndTable();
    }
    
    // Show which point was selected (if we can determine it)
    ImGui::Spacing();
    ImGui::Text("Note: Lower total score is better. The selected point has the lowest total score.");
} 