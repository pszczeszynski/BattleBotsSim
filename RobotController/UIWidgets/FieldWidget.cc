#include "FieldWidget.h"
#include "../RobotController.h"
#include "../GuiUtils.h"

FieldWidget::FieldWidget()
{
}

FieldWidget& FieldWidget::GetInstance()
{
    static FieldWidget instance;
    return instance;
}

void FieldWidget::Draw()
{
    // Start the main window without any specific flags
    ImGui::Begin("Field");

    // Then, for the content, create a child window that has the ImGuiWindowFlags_NoMove flag.
    ImGui::BeginChild("Field Content", ImVec2(0, 0), false, ImGuiWindowFlags_NoMove);

    // Get field image
    cv::Mat& fieldImage = RobotController::GetInstance().GetDrawingImage();
    // Convert mat to texture
    ImTextureID texture = MatToTexture(fieldImage);
    // Draw texture
    ImGui::Image(texture, ImVec2(fieldImage.cols, fieldImage.rows));

    // Get the coords of the above image
    _windowPos = ImGui::GetItemRectMin();

    // End child and main window
    ImGui::EndChild();
    ImGui::End();    
}


/**
 * Gets the mouse position relative to this window
*/
cv::Point2f FieldWidget::GetMousePosOnField()
{
    ImVec2 mousePosVec2 = ImGui::GetMousePos();
    cv::Point2f mousePosAbsolute{mousePosVec2.x, mousePosVec2.y};
    cv::Point2f windowPos = cv::Point2f(_windowPos.x, _windowPos.y);
    return mousePosAbsolute - windowPos;
}