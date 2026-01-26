#include "TrackingWidget.h"
#include "../RobotController.h"
#include "../RobotConfig.h"
#include "../TrackingEditorState.h"
#include "../Input/InputState.h"
#include "CameraWidget.h"
#include "../SafeDrawing.h"
#include "ColorScheme.h"

TrackingWidget *TrackingWidget::_instance = nullptr;
cv::Point2f TrackingWidget::robotMouseClickPoint = cv::Point2f(0, 0);
cv::Point2f TrackingWidget::opponentMouseClickPoint = cv::Point2f(0, 0);
double TrackingWidget::robotMouseClickAngle = 0;
double TrackingWidget::opponentMouseClickAngle = 0;


#define MASK_PATH "./backgrounds/fieldMask.jpg"

TrackingWidget::TrackingWidget() : // initialize the mask to all white
                                   _fieldMask{WIDTH, HEIGHT, CV_8UC1, cv::Scalar{0}},
                                   ImageWidget("Tracking", _trackingMat, false)
{
    _instance = this;

    std::filesystem::path dirPath = MASK_PATH;

    // if there is a mask file saved
    if (std::filesystem::exists(dirPath))
    {
        // read the last mask
        _fieldMask = cv::imread(MASK_PATH);
        if (_fieldMask.channels() == 3)
        {
            cv::cvtColor(_fieldMask, _fieldMask, cv::COLOR_BGR2GRAY);
        }
        else
        {
            _fieldMask = cv::Mat{WIDTH, HEIGHT, CV_8UC1, cv::Scalar{0}};
        }
    }

    // Initialize default colors for each variant using our color scheme
    variantColors["Camera"] = ColorScheme::GetVariantColor("Camera");
    variantOffsets["Camera"] = cv::Point(0, 0); // No offset for Camera

    variantColors["Blob"] = ColorScheme::GetVariantColor("Blob");
    variantOffsets["Blob"] = cv::Point(0, 0); // No offset for Blob

    variantColors["Heuristic"] = ColorScheme::GetVariantColor("Heuristic");
    variantOffsets["Heuristic"] = cv::Point(0, 0); // No offset for Heuristic

    variantColors["Neural"] = ImVec4(0.0f, 0.5f, 0.5f, 1.0f);   // Default: Cyan
    variantOffsets["Neural"] = cv::Point(0, 0); // No offset for Neural

    variantColors["Fusion"] = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);   // Default: Yellow
    variantOffsets["Fusion"] = cv::Point(0, 0); // No offset for Fusion

    variantColors["Opencv"] = ImVec4(0.2f, 0.2f, 0.2f, 1.0f);   // Default: Grey
    variantOffsets["Opencv"] = cv::Point(0, 0);

    variantColors["NeuralRot"] = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);   // Default: Grey
    variantOffsets["NeuralRot"] = cv::Point(0, 0);

    RestoreGUISettings(VISION_TRACKING_GUI);
    
}

void TrackingWidget::_GrabFrame()
{
    static int last_id = 0;

    // 1. populate the tracking mat (for display purposes)
    static ICameraReceiver *camera = ICameraReceiver::GetInstance();
    if (camera != nullptr && camera->NewFrameReady(last_id))
    {
        last_id = camera->GetFrame(GetDebugImage("Camera"), last_id);      
    }
}

void TrackingWidget::_AdjustFieldCrop()
{
    bool enableCrop = !CameraWidget::LockCamera;

    static int cornerToAdjust = -1;
    static cv::Point2f mousePosLast = cv::Point2f(0, 0);
    static cv::Point2f cornerHandles[4] = {cv::Point2f(0, 0),
                                           cv::Point2f(WIDTH, 0),
                                           cv::Point2f(WIDTH, HEIGHT),
                                           cv::Point2f(0, HEIGHT)};

    const double CORNER_DIST_THRESH = 20.0;

    // get the curr mouse position
    cv::Point2f currMousePos = GetMousePos();

    cv::Mat &drawingImage = _trackingMat;

    // draw the corners
    if (enableCrop && !drawingImage.empty())
    {
        for (int i = 0; i < 4; i++)
        {
            if (cv::norm(cornerHandles[i] - currMousePos) < CORNER_DIST_THRESH)
            {
                safe_circle(drawingImage, cornerHandles[i], CORNER_DIST_THRESH * 1.5, cv::Scalar(255, 100, 255), 2);
            }
            else
            {
                safe_circle(drawingImage, cornerHandles[i], CORNER_DIST_THRESH, cv::Scalar(255, 0, 255), 2);
            }
        }
    }

    // if the user isn't pressing shift and is over the image
    if (!InputState::GetInstance().IsKeyDown(ImGuiKey_LeftShift))
    {
        // corner adjustment

        // If the user left clicks near one of the corners
        if (enableCrop && InputState::GetInstance().IsMouseDown(0))
        {
            if (cornerToAdjust == -1)
            {
                // Check each corner
                for (int i = 0; i < 4; i++)
                {
                    // if the user is near a corner
                    if (cv::norm(cornerHandles[i] - currMousePos) < CORNER_DIST_THRESH)
                    {
                        // set the corner to adjust
                        cornerToAdjust = i;
                        break;
                    }
                }
            }
        }
        else
        {
            // otherwise set the corner to adjust to -1
            cornerToAdjust = -1;
        }

        // adjust the corner
        cv::Point2f adjustment = mousePosLast - currMousePos;

        if (cornerToAdjust == 0)
        {
            preprocess_tl_x += adjustment.x;
            preprocess_tl_y += adjustment.y;
        }
        else if (cornerToAdjust == 1)
        {
            preprocess_tr_x += adjustment.x;
            preprocess_tr_y += adjustment.y;
        }
        else if (cornerToAdjust == 2)
        {
            preprocess_br_x += adjustment.x;
            preprocess_br_y += adjustment.y;
        }
        else if (cornerToAdjust == 3)
        {
            preprocess_bl_x += adjustment.x;
            preprocess_bl_y += adjustment.y;
        }
    }

    // save the last mouse position
    mousePosLast = currMousePos;
}

TrackingWidget *TrackingWidget::GetInstance()
{
    return _instance;
}


enum class MaskState
{
    LOCKED = 0,
    WAITING_FOR_CLICK,
    DRAGGING
};

/**
 * Allows the user to mask out regions of the image by drawing rectangles.
 * Future camera sources will have these regions blacked out.
*/
void TrackingWidget::_MaskOutRegions()
{
    static MaskState state{MaskState::LOCKED};
    static cv::Point2f topLeft{0, 0};
    static cv::Point2f bottomRight{0, 0};

    // if user clicks checkbox to draw the mask, go to the waiting for click state
    if (CameraWidget::DrawMask)
    {
        if (state == MaskState::LOCKED)
        {
            state = MaskState::WAITING_FOR_CLICK;
        }
    }
    // otherwise go to locked
    else
    {
        state = MaskState::LOCKED;
    }

    // if waiting for a click
    if (state == MaskState::WAITING_FOR_CLICK)
    {
        // check if the left mouse button is down
        if (InputState::GetInstance().IsMouseDown(0) && IsMouseOver())
        {
            state = MaskState::DRAGGING;
            topLeft = GetMousePos();
        }
    }

    // if dragging the rectangle
    if (state == MaskState::DRAGGING)
    {
        bottomRight = GetMousePos();

        // check if the left mouse button is released
        if (!InputState::GetInstance().IsMouseDown(0))
        {
            // create rect with the top left and bottom right point
            cv::Rect rect = cv::Rect{topLeft, bottomRight};

            // draw the rect
            cv::rectangle(_fieldMask, cv::Rect{topLeft, bottomRight}, cv::Scalar(255), -1);
            // save the mask
            cv::imwrite(MASK_PATH, _fieldMask);

            // go back to waiting for click
            state = MaskState::WAITING_FOR_CLICK;
        }
    }
}

/**
 * Clears the field mask
*/
void TrackingWidget::ClearMask()
{
    // set the mask to all white
    _fieldMask = cv::Mat{WIDTH, HEIGHT, CV_8UC1, cv::Scalar{0}};
    cv::imwrite(MASK_PATH, _fieldMask);
}

cv::Mat& TrackingWidget::GetMask()
{
    return _fieldMask;
}

void DrawX(cv::Mat& mat, cv::Point2f pos, cv::Scalar color, int size)
{
    cv::line(mat, pos + cv::Point2f(-size, -size), pos + cv::Point2f(size, size), color, 2);
    cv::line(mat, pos + cv::Point2f(-size, size), pos + cv::Point2f(size, -size), color, 2);
}


void TrackingWidget::_DrawAlgorithmData()
{
    if (_trackingMat.empty())
    {
        return;
    }
    // draw arrow on opponent robot showing gamepad stick angle
    OdometryData opponentData = RobotController::GetInstance().odometry.Opponent();
    cv::Point2f opponentPos = opponentData.GetPositionOrZero();
    
    // Read gamepad right stick values
    float stickX = RobotController::GetInstance().gamepad.GetRightStickX();
    float stickY = RobotController::GetInstance().gamepad.GetRightStickY();
    
    // Calculate angle from stick position (only if stick is being moved)
    float stickMagnitude = sqrt(stickX * stickX + stickY * stickY);
    if (stickMagnitude > 0.1) {  // deadzone threshold
      float stickAngle = atan2(-stickY, stickX);
      cv::Point2f stickArrowEnd =
          opponentPos + cv::Point2f{cos(stickAngle), sin(stickAngle)} * 50;
      safe_arrow(_trackingMat, opponentPos, stickArrowEnd,
                 cv::Scalar(0, 255, 255), 3);
    }

    // The color format seems to be BLUE, GREEN, RED
    // blob is blue
    cv::Scalar blobColor = cv::Scalar(255, 0, 0);
    if( variantColors.find("Blob") != variantColors.end() )
    {
        blobColor = cv::Scalar(variantColors["Blob"].z * 255, variantColors["Blob"].y * 255, variantColors["Blob"].x * 255);
    }

    // neural is purple
    cv::Scalar neuralColor = cv::Scalar(255, 0, 255);
    if( variantColors.find("Neural") != variantColors.end() )
    {
        neuralColor = cv::Scalar(variantColors["Neural"].z * 255, variantColors["Neural"].y * 255, variantColors["Neural"].x * 255);
    }

    // heursitic is yellow - orange
    cv::Scalar heuristicColor = cv::Scalar(0, 180, 255);
    if( variantColors.find("Heuristic") != variantColors.end() )
    {
        heuristicColor = cv::Scalar(variantColors["Heuristic"].z * 255, variantColors["Heuristic"].y * 255, variantColors["Heuristic"].x * 255);
    }

    // opencv is red
    cv::Scalar opencvColor = cv::Scalar(0, 0, 255);
    if( variantColors.find("Opencv") != variantColors.end() )
    {
        opencvColor = cv::Scalar(variantColors["Opencv"].z * 255, variantColors["Opencv"].y * 255, variantColors["Opencv"].x * 255);
    }

    // Fusion color
    cv::Scalar fusionColor = cv::Scalar(255, 255, 255);
    if( variantColors.find("Fusion") != variantColors.end() )
    {
        fusionColor = cv::Scalar(variantColors["Fusion"].z * 255, variantColors["Fusion"].y * 255, variantColors["Fusion"].x * 255);
    }

    // LKFlow is cyan
    cv::Scalar lkFlowColor = cv::Scalar(255, 255, 0);
    if( variantColors.find("LKFlow") != variantColors.end() )
    {
        lkFlowColor = cv::Scalar(variantColors["LKFlow"].z * 255, variantColors["LKFlow"].y * 255, variantColors["LKFlow"].x * 255);
    }
    RobotOdometry &odometry = RobotController::GetInstance().odometry;
    BlobDetection &_odometry_Blob = odometry.GetBlobOdometry();
    HeuristicOdometry &_odometry_Heuristic = odometry.GetHeuristicOdometry();
    CVPosition& _odometry_Neural = odometry.GetNeuralOdometry();
    LKFlowTracker& _odometry_LKFlow = odometry.GetLKFlowOdometry();
#ifdef USE_OPENCV_TRACKER
    OpenCVTracker& _odometry_opencv = odometry.GetOpenCVOdometry();
#endif


    // go through every odometry algorithm and draw the tracking results
#ifdef USE_OPENCV_TRACKER
    if (_odometry_opencv.IsRunning() && showOpencv )
    {    
        _DrawPositions(_odometry_opencv.GetData(false) , _odometry_opencv.GetData(true), _trackingMat, opencvColor);  
        _DrawAngles(_odometry_opencv.GetData(false), _odometry_opencv.GetData(true), _trackingMat, opencvColor);
    }
#endif

    if (_odometry_Blob.IsRunning() && showBlob)
    {
        OdometryData blobData = _odometry_Blob.GetData(false);
        _DrawPositions(blobData, blobData, _trackingMat, blobColor);  
        _DrawAngles(blobData, blobData, _trackingMat, blobColor);
    }

    if (_odometry_Heuristic.IsRunning() && showHeuristic)
    {
        OdometryData heuristicData = _odometry_Heuristic.GetData(false);
        _DrawPositions(heuristicData, heuristicData, _trackingMat, heuristicColor);  
        _DrawAngles(heuristicData, heuristicData, _trackingMat, heuristicColor); 
    }

    if (_odometry_Neural.IsRunning() && showNeural)
    {
        OdometryData neuralData = _odometry_Neural.GetData(false);
        _DrawPositions(neuralData, neuralData, _trackingMat, neuralColor);
    }

    if (_odometry_LKFlow.IsRunning() && showLKFlow)
    {
        // LKFlow only tracks opponent, so robot data will be invalid
        OdometryData lkFlowData = _odometry_LKFlow.GetData(false);
        _DrawPositions(lkFlowData, lkFlowData, _trackingMat, lkFlowColor);
        _DrawAngles(lkFlowData, lkFlowData, _trackingMat, lkFlowColor);
    }

    // check if mouse is over the image
    if (IsMouseOver())
    {
        // if pressing not pressing shift
        if (!InputState::GetInstance().IsKeyDown(ImGuiKey_LeftShift))
        {
            if (InputState::GetInstance().IsMouseDown(0))
            {
                robotMouseClickPoint = GetMousePos();
                odometry.UpdateForceSetPosAndVel(robotMouseClickPoint, cv::Point2f(0,0), false);
            }
                        
            if (InputState::GetInstance().IsMouseDown(1))
            {
                opponentMouseClickPoint = GetMousePos();
                odometry.UpdateForceSetPosAndVel(opponentMouseClickPoint, cv::Point2f(0,0), true);
            }

            TrackingEditorState& editorState = TrackingEditorState::GetInstance();

#ifdef USE_OPENCV_TRACKER
            if (editorState.IsOpenCVEditing())
            {
                if (InputState::GetInstance().IsMouseDown(0))
                {
                    _odometry_opencv.SetPosition(GetMousePos(), false);
                }
                else if (InputState::GetInstance().IsMouseDown(1))
                {
                    _odometry_opencv.SetPosition(GetMousePos(), true);
                }
            }
#endif
            if (editorState.IsBlobEditing())
            {
                if (InputState::GetInstance().IsMouseDown(0))
                {
                    _odometry_Blob.SetPosition(GetMousePos(), false);
                    // force velocity to 0
                    _odometry_Blob.SetVelocity(cv::Point2f(0,0), false);
                }
                else if (InputState::GetInstance().IsMouseDown(1))
                {
                    _odometry_Blob.SetPosition(GetMousePos(), true);
                    _odometry_Blob.SetVelocity(cv::Point2f(0,0), true);
                }
            }
            if (editorState.IsHeuristicEditing())
            {
                if (InputState::GetInstance().IsMouseDown(0))
                {
                    if (InputState::GetInstance().IsKeyDown(ImGuiKey_LeftCtrl))
                    {
                        _odometry_Heuristic.SetPosition(GetMousePos(), false);
                    }
                    else
                    {
                        _odometry_Heuristic.ForcePosition(GetMousePos(), false);
                    }

                    _odometry_Heuristic.SetVelocity(cv::Point2f(0,0), false);
                }
                else if (InputState::GetInstance().IsMouseDown(1))
                {
                    if (InputState::GetInstance().IsKeyDown(ImGuiKey_RightCtrl))
                    {
                        _odometry_Heuristic.SetPosition(GetMousePos(), true);
                    }
                    else
                    {
                        _odometry_Heuristic.ForcePosition(GetMousePos(), true);
                    }
                    _odometry_Heuristic.SetVelocity(cv::Point2f(0,0), true);
                }
            }
        }
        // else pressing shift
        else
        {
            // if the user presses the left mouse button with shift
            if (InputState::GetInstance().IsMouseDown(0))
            {
                // set the robot angle
                cv::Point2f robotPos = odometry.Robot().GetPositionOrZero();
                cv::Point2f currMousePos = GetMousePos();
                double newAngle = atan2(currMousePos.y - robotPos.y, currMousePos.x - robotPos.x);
                odometry.UpdateForceSetAngle(newAngle, false);
                robotMouseClickAngle = newAngle;
            }
            else if (InputState::GetInstance().IsMouseDown(1))
            {
                // set the opponent angle
                cv::Point2f opponentPos = odometry.Opponent().GetPositionOrZero();
                cv::Point2f currMousePos = GetMousePos();
                double newAngle = atan2(currMousePos.y - opponentPos.y, currMousePos.x - opponentPos.x);
                odometry.UpdateForceSetAngle(newAngle, true);
                opponentMouseClickAngle = newAngle;
            }
        }
    }

    if( showFusion)
    {
        OdometryData robotData = odometry.Robot();
        OdometryData opponentData = odometry.Opponent();
        _DrawPositions(robotData, opponentData, _trackingMat, fusionColor);  
        _DrawAngles(robotData, opponentData, _trackingMat, fusionColor);
    }
}

void TrackingWidget::_DrawAngles(OdometryData& robot, OdometryData& opponent, cv::Mat& currMatt, cv::Scalar& arrowColor, bool forceShow)
{
    int linethickness = 1;

    if( robot.angle.has_value() || forceShow)
    {
        linethickness = 3;
    }

    // draw robot angle with arrow
    cv::Point2f robotPos = robot.GetPositionOrZero();
    double robotAngle = robot.GetAngleOrZero();
    cv::Point2f arrowEnd = robotPos + cv::Point2f(50 * cos(robotAngle), 50 * sin(robotAngle));
    safe_arrow(currMatt, robotPos, arrowEnd, arrowColor, linethickness);

    linethickness = 1;
    if (opponent.angle.has_value() || forceShow) {
      linethickness = 3;
    }

    // draw opponent angle with arrow
    cv::Point2f opponentPos = opponent.GetPositionOrZero();
    double opponentAngle = opponent.GetAngleOrZero();
    arrowEnd = opponentPos + cv::Point2f(50 * cos(opponentAngle), 50 * sin(opponentAngle));
    safe_arrow(currMatt, opponentPos, arrowEnd, arrowColor, linethickness);
    

}

void TrackingWidget::_DrawPositions(OdometryData& robot, OdometryData& opponent, cv::Mat& currMatt, cv::Scalar& arrowColor, bool forceShow)
{
    int size = 5;

    if (robot.pos.has_value() || forceShow) {
      size = 20;
    }

    OdometryData robot_ext =
        robot.ExtrapolateBoundedTo(Clock::programClock.getElapsedTime());
    DrawX(currMatt, robot_ext.GetPositionOrZero(), arrowColor, size);

    size = 5;
    if (opponent.pos.has_value() || forceShow) {
      size = 20;
    }

    OdometryData opponent_ext = opponent.ExtrapolateBoundedTo(Clock::programClock.getElapsedTime());
    safe_circle(currMatt, opponent_ext.GetPositionOrZero(), size, arrowColor, 2);
}


cv::Mat& TrackingWidget::GetTrackingMat()
{
    return _trackingMat;
}

void TrackingWidget::Update()
{
    // Get the updated camera frame
    _GrabFrame();

    // Render all the frames together
    _RenderFrames();

    _AdjustFieldCrop();
    // Allow the user to mask out areas of the image.
    // They will be set to black when you get a frame from a camera
    _MaskOutRegions();

    _DrawAlgorithmData();

    UpdateMat(_trackingMat);

    // Save to video (if enabled)
    SaveToVideo();
}

// New function to store an image with a given label
// The image should be a clone of the original image to avoid issues with references
void TrackingWidget::UpdateDebugImage(const std::string& label, const cv::Mat& image) 
{
    variantImages[label] = image;
}

// Returns the old debug image, creates a new one if it doesn't exist
cv::Mat& TrackingWidget::GetDebugImage(const std::string& label)   
{
    if (variantImages.find(label) == variantImages.end())
    {
        variantImages[label] = cv::Mat(HEIGHT, WIDTH, CV_8UC1, cv::Scalar(0)); // Create an empty image if it doesn't exist
    }

    return variantImages[label];
}

cv::Point TrackingWidget::GetDebugOffset(const std::string& label)
{
    auto it = variantOffsets.find(label);
    if (it == variantOffsets.end())
    {
         variantOffsets[label] = cv::Point(0, 0); // Initialize with default offset if not found
    }

    return variantOffsets[label]; // Default offset if not found
}

void TrackingWidget::_DrawShowButton(const char* label, bool& enabledFlag) 
{
    ImGui::PushID(label); // Ensure unique IDs for widgets

    // Get the color for this variant (default to white if not found)
    ImVec4 color = variantColors.count(label) ? variantColors[label] : ImVec4(1.0f, 1.0f, 1.0f, 1.0f);

    // Apply button colors based on enabled state
    if (enabledFlag) {
        ColorScheme::PushSuccessColors();
    } else {
        ColorScheme::PushButtonColors(ColorScheme::PRIMARY_BLUE);
    }

    // Draw the toggle button
    if (ImGui::Button(enabledFlag ? "Shown" : "Hidden")) {
        enabledFlag = !enabledFlag;
    }

    ColorScheme::PopButtonColors(); // Pop the button colors

    // Draw the label
    ImGui::SameLine();
    ImGui::Text(label);

    // Add a color picker for this variant
    ImGui::SameLine();
    float colorArray[4] = {color.x, color.y, color.z, color.w};
    if (ImGui::ColorEdit4(("Color##" + std::string(label)).c_str(), colorArray, ImGuiColorEditFlags_NoInputs)) {
        variantColors[label] = ImVec4(colorArray[0], colorArray[1], colorArray[2], colorArray[3]);
    }

    // Add x, y offset inputs using DragInt2 for integers
    ImGui::SameLine();
    ImGui::PushItemWidth(60); // Set width for input fields
    cv::Point offset = variantOffsets.count(label) ? variantOffsets[label] : cv::Point(0, 0);
    int offsetArray[2] = {offset.x, offset.y};
    if (ImGui::DragInt2(("Offset##" + std::string(label)).c_str(), offsetArray, 1, -1000, 1000)) {
        // Update stored offset if changed
        variantOffsets[label] = cv::Point(offsetArray[0], offsetArray[1]);
    }
    ImGui::PopItemWidth();

    ImGui::PopID();
}


void TrackingWidget::_RenderFrames()
{
    // Initialize the output image as empty
    _trackingMat = cv::Mat();

    // List of variant labels and their corresponding visibility flags
    std::vector<std::pair<std::string, bool>> variants = {
        {"Camera", showCamera},
        {"Blob", showBlob},
        {"Heuristic", showHeuristic},
        {"Neural", showNeural},
        {"Fusion", showFusion},
        {"NeuralRot", showNeuralRot},
#ifdef USE_OPENCV_TRACKER
        {"Opencv", showOpencv},
#endif
        {"LKFlow", showLKFlow}
    };

    // Use the camera image size as the output size
    cv::Size outputSize = GetDebugImage("Camera").size();

    // Initialize output image as a black color image (CV_8UC3)
    _trackingMat = cv::Mat::zeros(outputSize, CV_8UC3);

    // Pre-allocate working matrices to avoid repeated allocations
    cv::Mat colorized, mask, temp3channel;

    // Iterate through each variant to overlay images
    for (const auto& variant : variants) {
        const std::string& label = variant.first;
        bool isVisible = variant.second;

        // Skip if the variant is not visible or the image doesn't exist
        if (!isVisible || variantImages.find(label) == variantImages.end()) {
            continue;
        }

        const cv::Mat& srcImage = variantImages[label];
        // Skip empty or invalid images
        if (srcImage.empty() || srcImage.cols != outputSize.width || srcImage.rows != outputSize.height) {
            continue;
        }

        // Get the color for this variant (default to white if not found)
        ImVec4 color = variantColors.count(label) ? variantColors[label] : ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
        cv::Scalar bgrColor(color.z * 255, color.y * 255, color.x * 255); // Convert RGB to BGR for OpenCV
        float alpha = color.w; // Use the alpha value from ImVec4

        // Create a colorized version of the source image using vectorized operations
        if (srcImage.type() == CV_8UC1) { // Grayscale or binary mask
            // Convert grayscale to 3-channel
            cv::cvtColor(srcImage, temp3channel, cv::COLOR_GRAY2BGR);
            
            // Create colored version by multiplying each channel
            std::vector<cv::Mat> channels(3);
            cv::split(temp3channel, channels);
            
            // Apply color scaling to each channel using vectorized operations
            channels[0] *= (bgrColor[0] / 255.0); // Blue
            channels[1] *= (bgrColor[1] / 255.0); // Green  
            channels[2] *= (bgrColor[2] / 255.0); // Red
            
            cv::merge(channels, colorized);
            
            // Create mask for non-zero pixels
            cv::threshold(srcImage, mask, 0, 255, cv::THRESH_BINARY);
            
        } else if (srcImage.type() == CV_8UC3) { // Already a color image
            // Apply color tint using vectorized multiplication
            srcImage.copyTo(colorized);
            
            std::vector<cv::Mat> channels(3);
            cv::split(colorized, channels);
            
            // Apply color scaling to each channel
            channels[0] *= (bgrColor[0] / 255.0); // Blue
            channels[1] *= (bgrColor[1] / 255.0); // Green
            channels[2] *= (bgrColor[2] / 255.0); // Red
            
            cv::merge(channels, colorized);
            
            // Create mask for non-zero pixels (any channel > 0)
            cv::Mat grayTemp;
            cv::cvtColor(srcImage, grayTemp, cv::COLOR_BGR2GRAY);
            cv::threshold(grayTemp, mask, 0, 255, cv::THRESH_BINARY);
            
        } else {
            continue; // Unsupported image type
        }

        // Blend using OpenCV's optimized functions
        if (alpha >= 0.99f) {
            // Full opacity - just copy over the mask
            colorized.copyTo(_trackingMat, mask);
        } else {
            // Partial opacity - use addWeighted for blending
            cv::Mat maskedColorized = cv::Mat::zeros(colorized.size(), colorized.type());
            colorized.copyTo(maskedColorized, mask);
            
            // Create inverse mask for existing content
            cv::Mat inverseMask;
            cv::bitwise_not(mask, inverseMask);
            
            cv::Mat existing = cv::Mat::zeros(_trackingMat.size(), _trackingMat.type());
            _trackingMat.copyTo(existing, inverseMask);
            
            // Blend the regions
            cv::Mat blended;
            cv::addWeighted(_trackingMat, 1.0 - alpha, maskedColorized, alpha, 0, blended);
            
            // Copy back only the masked region
            blended.copyTo(_trackingMat, mask);
        }
    }
}

void TrackingWidget::AutoMatchStart(bool left)
{
    // Set leftstart and rightstart to middle of background starting boxes
    cv::Point2f leftStart = cv::Point2f((STARTING_LEFT_TL_x+STARTING_LEFT_BR_x)/2, (STARTING_LEFT_TL_y+STARTING_LEFT_BR_y)/2);
    cv::Point2f rightStart = cv::Point2f((STARTING_RIGHT_TL_x+STARTING_RIGHT_BR_x)/2, (STARTING_RIGHT_TL_y+STARTING_RIGHT_BR_y)/2);
    
    // Initialize left/right click positions and angles to center of our starting rectangles
    if( left)
    {
        robotMouseClickPoint = leftStart;
        robotMouseClickAngle = deg2rad(0);
        opponentMouseClickPoint = rightStart;
        opponentMouseClickAngle = deg2rad(-180.0f);
    }
    else
    {
        robotMouseClickPoint = rightStart;
        robotMouseClickAngle = deg2rad(-180.0f);
        opponentMouseClickPoint = leftStart;
        opponentMouseClickAngle = deg2rad(0);
    }

    RobotController::GetInstance().odometry.AutoMatchStart();
}

void TrackingWidget::DrawGUI() {
    ImGui::Begin("Tracking Config");

    // Draw buttons and inputs for each variant
    _DrawShowButton("Camera", showCamera);
    _DrawShowButton("Blob", showBlob);
    _DrawShowButton("Heuristic", showHeuristic);
    _DrawShowButton("Neural", showNeural);
    _DrawShowButton("NeuralRot", showNeuralRot);
#ifdef USE_OPENCV_TRACKER
    _DrawShowButton("Opencv", showOpencv);
#endif
    _DrawShowButton("LKFlow", showLKFlow);
    _DrawShowButton("Fusion", showFusion);

    // Add two line spacers
    ImGui::Separator();

    ImGui::Dummy(ImVec2(0.0f, 30.0f));
    
    // ADD Heuristic status line
    ImGui::SetWindowFontScale(1.5f);

    // Create a child window with fixed height for the text
    ImGui::BeginChild("HeuristicStatus", ImVec2(0, 60.0f), false); // 60 pixels height, adjust as needed
    ImGui::TextWrapped("Heuristic Status: %s", HeuristicOdometry::statusstring.c_str());
    ImGui::EndChild();

    ImGui::SetWindowFontScale(1.0f);

    // Remove ImGui::Dummy, as the child window handles spacing
    ImGui::Dummy(ImVec2(0.0f, 30.0f));
 
    RobotOdometry& odometry = RobotController::GetInstance().odometry;

    HeuristicOdometry& heuristic = odometry.GetHeuristicOdometry();

    // Set leftstart and rightstart to middle of background starting boxes
    cv::Point2f leftStart = cv::Point2f((STARTING_LEFT_TL_x+STARTING_LEFT_BR_x)/2, (STARTING_LEFT_TL_y+STARTING_LEFT_BR_y)/2);
    cv::Point2f rightStart = cv::Point2f((STARTING_RIGHT_TL_x+STARTING_RIGHT_BR_x)/2, (STARTING_RIGHT_TL_y+STARTING_RIGHT_BR_y)/2);

    ImGui::PushStyleColor(ImGuiCol_Button, ColorScheme::PRIMARY_BLUE);
    if (ImGui::Button("Auto Start Left", ImVec2(150, 40)))
    {
        AutoMatchStart(true);

    }
    ImGui::PopStyleColor();

    ImGui::SameLine();
    ImGui::Text("        ");
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ColorScheme::PRIMARY_RED);
    if (ImGui::Button("Auto Start Right", ImVec2(150, 40)))
    {
        AutoMatchStart(false);
    }

    ImGui::PopStyleColor();

    ImGui::PushStyleColor(ImGuiCol_Button, ColorScheme::PRIMARY_BLUE);
    if (ImGui::Button("Start Left", ImVec2(150, 40)))
    {
        odometry.MatchStart();
    }
    ImGui::PopStyleColor();

    ImGui::SameLine();
    ImGui::Text("        ");
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ColorScheme::PRIMARY_RED);
    if (ImGui::Button("Start Right", ImVec2(150, 40)))
    {
        odometry.MatchStart();
    }
    ImGui::PopStyleColor();


    ImGui::Dummy(ImVec2(0.0f, 20.0f));

    ImGui::Text("   ");
    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, ColorScheme::PRIMARY_BLUE);
    if (ImGui::Button("Auto Recover", ImVec2(150, 40)))
    {
        heuristic.RecoverDetection();

    }
    ImGui::PopStyleColor();

    





    ImGui::Dummy(ImVec2(0.0f, 80.0f));


    // Add input field for outputVideoFile
    ImGui::Text("Output Video File:");
    ImGui::InputText("##trackvid", outputVideoFileBuffer, sizeof(outputVideoFileBuffer));
    outputVideoFile = outputVideoFileBuffer; // Update std::string from buffer

    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    bool doPopStyle = false;
    // Add button to toggle save_video_enabled
    if (save_video_enabled) {
        // Push red color for button background when recording
        ColorScheme::PushErrorColors();
        doPopStyle = true;
    }
    if (ImGui::Button(save_video_enabled ? "Stop Recording" : "Start Recording")) {
        save_video_enabled = !save_video_enabled; // Toggle the boolean
    }

    if (doPopStyle) {
        // Pop the custom color after rendering the button
        ColorScheme::PopStatusColors();
    }




    ImGui::End();
}

std::string TrackingWidget::SaveGUISettings() {
    std::stringstream ss;

    // Save boolean flags
    ss << "showCamera=" << (showCamera ? "1" : "0") << ";";
    ss << "showBlob=" << (showBlob ? "1" : "0") << ";";
    ss << "showHeuristic=" << (showHeuristic ? "1" : "0") << ";";
    ss << "showNeural=" << (showNeural ? "1" : "0") << ";";
    ss << "showNeuralRot=" << (showNeuralRot ? "1" : "0") << ";";
    ss << "showFusion=" << (showFusion ? "1" : "0") << ";";
    ss << "showOpencv=" << (showOpencv ? "1" : "0") << ";";
    ss << "showLKFlow=" << (showLKFlow ? "1" : "0") << ";";


    // Save outputVideoFile
    ss << "outputVideoFile=" << outputVideoFile << ";";

    // Save variantColors
    ss << "variantColors=";
    for (const auto& pair : variantColors) {
        ss << pair.first << ":" 
           << pair.second.x << "," 
           << pair.second.y << "," 
           << pair.second.z << "," 
           << pair.second.w << "|";
    }
    ss << ";";

    // Save variantOffsets
    ss << "variantOffsets=";
    for (const auto& pair : variantOffsets) {
        ss << pair.first << ":" 
           << pair.second.x << "," 
           << pair.second.y << "|";
    }
    ss << ";";

    return ss.str();
}

void TrackingWidget::RestoreGUISettings(const std::string& settings) {
    std::stringstream ss(settings);
    std::string token;

    // Helper function to split a string by delimiter
    auto split = [](const std::string& s, char delim) -> std::vector<std::string> {
        std::vector<std::string> result;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim)) {
            result.push_back(item);
        }
        return result;
    };

    while (std::getline(ss, token, ';')) {
        if (token.empty()) continue;
        auto keyValue = split(token, '=');
        if (keyValue.size() != 2) continue;
        const std::string& key = keyValue[0];
        const std::string& value = keyValue[1];

        // Restore boolean flags
        if (key == "showCamera") showCamera = (value == "1");
        else if (key == "showBlob") showBlob = (value == "1");
        else if (key == "showHeuristic") showHeuristic = (value == "1");
        else if (key == "showNeural") showNeural = (value == "1");
        else if (key == "showNeuralRot") showNeuralRot = (value == "1");
        else if (key == "showFusion") showFusion = (value == "1");
        else if (key == "showOpencv") showOpencv = (value == "1");
        else if (key == "showLKFlow") showLKFlow = (value == "1");


        // Restore outputVideoFile
        else if (key == "outputVideoFile") {
            outputVideoFile = value;
            strncpy(outputVideoFileBuffer, value.c_str(), sizeof(outputVideoFileBuffer) - 1);
            outputVideoFileBuffer[sizeof(outputVideoFileBuffer) - 1] = '\0'; // Ensure null-termination
        }

        // Restore variantColors
        else if (key == "variantColors") {
            variantColors.clear();
            auto colorEntries = split(value, '|');
            for (const auto& entry : colorEntries) {
                if (entry.empty()) continue;
                auto colorData = split(entry, ':');
                if (colorData.size() != 2) continue;
                auto colors = split(colorData[1], ',');
                if (colors.size() != 4) continue;
                variantColors[colorData[0]] = ImVec4(
                    std::stof(colors[0]),
                    std::stof(colors[1]),
                    std::stof(colors[2]),
                    std::stof(colors[3])
                );
            }
        }

        // Restore variantOffsets
        else if (key == "variantOffsets") {
            variantOffsets.clear();
            auto offsetEntries = split(value, '|');
            for (const auto& entry : offsetEntries) {
                if (entry.empty()) continue;
                auto offsetData = split(entry, ':');
                if (offsetData.size() != 2) continue;
                auto offsets = split(offsetData[1], ',');
                if (offsets.size() != 2) continue;
                variantOffsets[offsetData[0]] = cv::Point(
                    std::stoi(offsets[0]),
                    std::stoi(offsets[1])
                );
            }
        }
    }
}

void TrackingWidget::SaveToVideo()
{
    if (!save_video_enabled)
    {
        if( save_video_enabled_old && video.isOpened()) 
        {
            // If we were previously saving, close the video file
            video.release();
        }

        save_video_enabled_old = false;
        return;
    }


    // Try avc1 firST
    if (!video.isOpened())
    {
        int fourcc = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
        video.open(outputVideoFile, fourcc, 60.0,cv::Size(_trackingMat.cols, _trackingMat.rows), true);
    }

    // Make sure we are initialized
    if (!video.isOpened())
    {
        // Try XVID codec
        int fourcc =  cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
        video.open(outputVideoFile, fourcc, 60.0,cv::Size(_trackingMat.cols, _trackingMat.rows), true);
        if (!video.isOpened()) 
        {
            // Fallback to MJPG
            fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
            video.open(outputVideoFile, fourcc, 60.0,cv::Size(_trackingMat.cols, _trackingMat.rows), true);
            if (!video.isOpened()) 
            {
                save_video_enabled = false;
                return;
            }
        }
     }

    save_video_enabled_old = true;
    video.write(_trackingMat);
}



