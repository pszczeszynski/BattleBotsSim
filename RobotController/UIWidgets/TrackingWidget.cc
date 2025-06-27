#include "TrackingWidget.h"
#include "../RobotController.h"
#include "../GuiUtils.h"
#include "../RobotConfig.h"
#include "../Input/InputState.h"
#include "CameraWidget.h"
#include "../SafeDrawing.h"

TrackingWidget *TrackingWidget::_instance = nullptr;
cv::Point2f TrackingWidget::leftClickPoint = cv::Point2f(0, 0);
cv::Point2f TrackingWidget::rightClickPoint = cv::Point2f(0, 0);


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

    // Initialize default colors for each variant
    // Using ImVec4 (RGBA), with alpha = 1.0f for opaque colors
    variantColors["Camera"] = ImVec4(0.0f, 0.7f, 0.0f, 1.0f); // Default: Green
    variantOffsets["Camera"] = cv::Point(0, 0); // No offset for Camera

    variantColors["Blob"] = ImVec4(0.0f, 0.0f, 0.5f, 1.0f);   // Default: Blue
    variantOffsets["Blob"] = cv::Point(0, 0); // No offset for Blob

    variantColors["Heuristic"] = ImVec4(0.8f, 0.0f, 0.0f, 1.0f); // Default: Red
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
    static ICameraReceiver &camera = ICameraReceiver::GetInstance();
    if (camera.NewFrameReady(last_id))
    {
        last_id = camera.GetFrame(GetDebugImage("Camera"), last_id);      
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
    // blob is blue
    cv::Scalar blobColor = cv::Scalar(255, 0, 0);

    // neural is purple
    cv::Scalar neuralColor = cv::Scalar(255, 0, 255);

    // heursitic is yellow - orange
    cv::Scalar heuristicColor = cv::Scalar(0, 180, 255);

    // opencv is red
    cv::Scalar opencvColor = cv::Scalar(0, 0, 255);

    RobotOdometry &odometry = RobotController::GetInstance().odometry;
    BlobDetection &_odometry_Blob = odometry.GetBlobOdometry();
    HeuristicOdometry &_odometry_Heuristic = odometry.GetHeuristicOdometry();
    CVPosition& _odometry_Neural = odometry.GetNeuralOdometry();
    OpenCVTracker& _odometry_opencv = odometry.GetOpenCVOdometry();


    // go through every odometry algorithm and draw the tracking results
    if (_odometry_opencv.IsRunning())
    {
        OdometryData robot = _odometry_opencv.GetData(false);
        robot.Extrapolate(Clock::programClock.getElapsedTime());
        DrawX(_trackingMat, robot.robotPosition, opencvColor, 20);

        OdometryData opponent = _odometry_opencv.GetData(true);
        opponent.Extrapolate(Clock::programClock.getElapsedTime());

        safe_circle(_trackingMat, opponent.robotPosition, 20, opencvColor, 2);
    }

    if (_odometry_Blob.IsRunning())
    {
        OdometryData robot = _odometry_Blob.GetData(false);
        robot.Extrapolate(Clock::programClock.getElapsedTime());
        DrawX(_trackingMat, robot.robotPosition, blobColor, (robot.robotPosValid) ? 20 : 5);

        OdometryData opponent = _odometry_Blob.GetData(true);
        opponent.Extrapolate(Clock::programClock.getElapsedTime());

        safe_circle(_trackingMat, opponent.robotPosition, (opponent.robotPosValid) ? 20 : 5, blobColor, 2);
    }

    if (_odometry_Heuristic.IsRunning())
    {
        OdometryData robot = _odometry_Heuristic.GetData(false);
        robot.Extrapolate(Clock::programClock.getElapsedTime());

        if (robot.robotPosValid)
        {
            DrawX(_trackingMat, robot.robotPosition, heuristicColor, (robot.robotPosValid) ? 20 : 5);
        }


        OdometryData opponent = _odometry_Heuristic.GetData(true);
        opponent.Extrapolate(Clock::programClock.getElapsedTime());

        if (opponent.robotAngleValid)
        {
            cv::Point2f arrowEnd = opponent.robotPosition + cv::Point2f(50 * cos(opponent.robotAngle), 50 * sin(opponent.robotAngle));
            safe_arrow(_trackingMat, opponent.robotPosition, arrowEnd, heuristicColor, 2);
        }

        if (opponent.robotPosValid)
        {
            safe_circle(_trackingMat, opponent.robotPosition, (opponent.robotPosValid) ? 20 : 5, heuristicColor, 2);
        }
    }

    if (_odometry_Neural.IsRunning())
    {
        OdometryData robot = _odometry_Neural.GetData(false);
        robot.Extrapolate(Clock::programClock.getElapsedTime());

        if( robot.robotPosValid && (robot.GetAge() < 0.3) )
        {
            DrawX(_trackingMat, robot.robotPosition, neuralColor, 30);
        }
       
    }


    // check if mouse is over the image
    if (IsMouseOver())
    {
        // if pressing not pressing shift
        if (!InputState::GetInstance().IsKeyDown(ImGuiKey_LeftShift))
        {
            if (EDITING_OPENCV)
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
            if (EDITING_BLOB)
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
            if (EDITING_HEU)
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
                cv::Point2f robotPos = odometry.Robot().robotPosition;
                cv::Point2f currMousePos = GetMousePos();
                double newAngle = atan2(currMousePos.y - robotPos.y, currMousePos.x - robotPos.x);
                odometry.UpdateForceSetAngle(newAngle, false);
            }
        }
    }


    // draw robot angle with arrow
    cv::Point2f robotPos = odometry.Robot().robotPosition;
    double robotAngle = odometry.Robot().robotAngle;
    cv::Point2f arrowEnd = robotPos + cv::Point2f(50 * cos(robotAngle), 50 * sin(robotAngle));
    safe_arrow(_trackingMat, robotPos, arrowEnd, cv::Scalar(255, 0, 0), 2);

    // draw opponent angle with arrow
    cv::Point2f opponentPos = odometry.Opponent().robotPosition;
    double opponentAngle = odometry.Opponent().robotAngle;
    arrowEnd = opponentPos + cv::Point2f(50 * cos(opponentAngle), 50 * sin(opponentAngle));
    safe_arrow(_trackingMat, opponentPos, arrowEnd, cv::Scalar(0, 0, 255), 2);
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
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.7f, 0.0f, 1.0f));        // Green
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.7f, 0.2f, 1.0f)); // Slightly lighter green
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));  // Darker green
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.0f, 0.7f, 1.0f));        // Blue
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.2f, 0.7f, 1.0f)); // Slightly lighter blue
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.0f, 0.0f, 0.5f, 1.0f));  // Darker blue
    }

    // Draw the toggle button
    if (ImGui::Button(enabledFlag ? "Shown" : "Hidden")) {
        enabledFlag = !enabledFlag;
    }

    ImGui::PopStyleColor(3); // Pop the button colors

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

void TrackingWidget::DrawGUI() {
    ImGui::Begin("Tracking Config");

    // Draw buttons and inputs for each variant
    _DrawShowButton("Camera", showCamera);
    _DrawShowButton("Blob", showBlob);
    _DrawShowButton("Heuristic", showHeuristic);
    _DrawShowButton("Neural", showNeural);
    _DrawShowButton("NeuralRot", showNeuralRot);
    _DrawShowButton("Opencv", showOpencv);
    _DrawShowButton("Fusion", showFusion);


    ImGui::End();
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
        {"Opencv", showOpencv}  
    };

    // Use the camera image size as the output size
    cv::Size outputSize = GetDebugImage("Camera").size();

    // Initialize output image as a black color image (CV_8UC3)
    _trackingMat = cv::Mat::zeros(outputSize, CV_8UC3);

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

        // Create a colorized version of the source image
        cv::Mat colorized;
        if (srcImage.type() == CV_8UC1) { // Grayscale or binary mask
            // Convert single-channel image to 3-channel color image
            cv::Mat temp;
            cv::cvtColor(srcImage, temp, cv::COLOR_GRAY2BGR);
            colorized = cv::Mat::zeros(srcImage.size(), CV_8UC3);
            for (int y = 0; y < srcImage.rows; y++) {
                for (int x = 0; x < srcImage.cols; x++) {
                    uchar intensity = srcImage.at<uchar>(y, x);
                    if (intensity > 0) { // Apply color only to non-zero pixels
                        colorized.at<cv::Vec3b>(y, x) = cv::Vec3b(
                            bgrColor[0] * (intensity / 255.0),
                            bgrColor[1] * (intensity / 255.0),
                            bgrColor[2] * (intensity / 255.0)
                        );
                    }
                }
            }
        } else if (srcImage.type() == CV_8UC3) { // Already a color image
            colorized = srcImage.clone(); // Use as is or apply color tint if needed
            // Optionally apply color tint (multiply with color)
            for (int y = 0; y < srcImage.rows; y++) {
                for (int x = 0; x < srcImage.cols; x++) {
                    cv::Vec3b pixel = colorized.at<cv::Vec3b>(y, x);
                    colorized.at<cv::Vec3b>(y, x) = cv::Vec3b(
                        pixel[0] * (bgrColor[0] / 255.0),
                        pixel[1] * (bgrColor[1] / 255.0),
                        pixel[2] * (bgrColor[2] / 255.0)
                    );
                }
            }
        } else {
            continue; // Unsupported image type
        }

        // Blend the colorized image onto the output using alpha
        float alpha = color.w; // Use the alpha value from ImVec4
        for (int y = 0; y < _trackingMat.rows; y++) {
            for (int x = 0; x < _trackingMat.cols; x++) {
                cv::Vec3b& outPixel = _trackingMat.at<cv::Vec3b>(y, x);
                cv::Vec3b srcPixel = colorized.at<cv::Vec3b>(y, x);
                // Blend only if the source pixel is non-zero
                if (srcPixel[0] > 0 || srcPixel[1] > 0 || srcPixel[2] > 0) {
                    outPixel = cv::Vec3b(
                        outPixel[0] * (1.0 - alpha) + srcPixel[0] * alpha,
                        outPixel[1] * (1.0 - alpha) + srcPixel[1] * alpha,
                        outPixel[2] * (1.0 - alpha) + srcPixel[2] * alpha
                    );
                }
            }
        }
    }
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



