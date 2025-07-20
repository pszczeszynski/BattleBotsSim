#pragma once

#include <opencv2/opencv.hpp>
#include "imgui.h"
#include "ImageWidget.h"

#define CURVE_WIDGET_WIDTH 300
#define CURVE_WIDGET_HEIGHT 200

class AStarAttackWidget : public ImageWidget
{
public:
    AStarAttackWidget();
    void Draw() override;
    void DrawGUI();
    void SyncFromAStarAttack(); // Sync parameters from AStarAttack
    
    static AStarAttackWidget* GetInstance();

private:
    void _DrawCurveVisualization();
    void _DrawRadiusCurveSliders();
    void _DrawPurePursuitSliders();
    void _UpdateAStarParameters();
    
    // Widget state
    static AStarAttackWidget* _instance;
    
    // Current parameter values (local copies for sliders)
    float _radiusCurveX[3] = {0.0f, 15.0f, 100.0f};
    float _radiusCurveY[3] = {0.0f, 85.0f, 150.0f};
    
    // Display settings
    bool _showGrid = true;
    float _maxX = 120.0f;
    float _maxY = 200.0f;
}; 