#pragma once

#include <opencv2/opencv.hpp>
#include "imgui.h"
#include "ImageWidget.h"
#include "../Globals.h"


/**
 * This is the widget for correcting + adjusting the odometry
 * tracking algorithms + the field warping
 */

class TrackingWidget : public ImageWidget
{
public:
    TrackingWidget();
    void ClearMask();
    cv::Mat& GetMask();
    cv::Mat& GetTrackingMat();
    void Update();
    void DrawGUI();

    static TrackingWidget* GetInstance();
    static cv::Point2f leftClickPoint;
    static cv::Point2f rightClickPoint;

    void UpdateDebugImage(const std::string& label, const cv::Mat& image);
    cv::Mat& GetDebugImage(const std::string& label);
    cv::Point GetDebugOffset(const std::string& label);
    std::string SaveGUISettings();
    void RestoreGUISettings(const std::string& settings);

    bool save_video_enabled = false;
    std::string outputVideoFile = "Recordings/Tracking_dataDump.mp4";

private:
    void _GrabFrame();
    void _DrawAlgorithmData();
    void _AdjustFieldCrop();
    void _MaskOutRegions();
    void _RenderFrames();

    static TrackingWidget* _instance;

    cv::Mat _fieldMask;
    cv::Mat _trackingMat{WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0)};

    bool showCamera =true;
    bool showBlob = false;
    bool showHeuristic = false;
    bool showNeural = false;
    bool showNeuralRot = false;
    bool showFusion = false;
    bool showOpencv = false;

    // Store colors for each variant (label -> ImVec4)
    std::unordered_map<std::string, ImVec4> variantColors;

    // Store images for each variant
    std::unordered_map<std::string, cv::Mat> variantImages;

        // Store offsets for each variant
     std::map<std::string, cv::Point> variantOffsets;


    void _DrawShowButton(const char* label, bool& enabledFlag);

    // Dump to video functions
    char outputVideoFileBuffer[256] = ""; // Buffer for ImGui text input
    cv::VideoWriter video;
    bool save_video_enabled_old = false;
    void SaveToVideo();

};