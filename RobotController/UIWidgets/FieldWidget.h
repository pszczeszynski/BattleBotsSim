#pragma once

#include <opencv2/opencv.hpp>
#include "imgui.h"
#include "ImageWidget.h"

class FieldWidget : public ImageWidget
{
public:
    FieldWidget();
    static FieldWidget* GetInstance();
    void DrawFieldBoundaryEditor();
    void DrawGUI();
    void LoadFieldBoundariesIfExists();
    
    // Field editing controls
    static bool EditFieldBoundaries;
    
private:
    void _DrawFieldBoundaryPoints();
    void _HandleFieldBoundaryInteraction();
    void _SaveFieldBoundaries(const std::string& filename);
    void _LoadFieldBoundaries(const std::string& filename);
    
    // Field boundary editing state
    int _selectedPointIndex = -1;
    bool _draggingPoint = false;
    cv::Point2f _lastMousePos = cv::Point2f(0, 0);
    static bool _boundariesLoaded;
    static bool _boundariesLoadedFromFile;
    
    static FieldWidget* _instance;
};