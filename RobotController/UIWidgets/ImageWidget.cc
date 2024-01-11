#include "ImageWidget.h"
#include "../RobotController.h"
#include "../GuiUtils.h"

/**
 * Constructor
*/
ImageWidget::ImageWidget(std::string name, bool moveable)
    : _name(name), _moveable(moveable)
{
    // add this instance to the list of instances
    Instances().push_back(this);
}

/**
 * Overload constructor with image
 */

ImageWidget::ImageWidget(std::string name, cv::Mat &image, bool moveable)
    : ImageWidget(name, moveable)
{
    // update the image
    UpdateMat(image);
}

/**
 * Destructor
*/
ImageWidget::~ImageWidget()
{
    // remove this instance from the list of instances
    std::vector<ImageWidget *> &instances = Instances();
    instances.erase(std::remove(instances.begin(), instances.end(), this), instances.end());
}

/**
 * Gets the list of all instances of this class
*/
std::vector<ImageWidget*>& ImageWidget::Instances()
{
    static std::vector<ImageWidget*> instances;
    return instances;
}

/**
 * Updates the image of the widget. Thread safe
*/
void ImageWidget::UpdateMat(cv::Mat& image)
{
    _imageMutex.lock();
    image.copyTo(_image);
    _imageMutex.unlock();
}

/**
 * Draws the widget
 */
void ImageWidget::Draw()
{
    // make sure not to draw if the image is empty
    if (_image.empty())
    {
        return;
    }

    // Start the main window without any specific flags
    ImGui::Begin(_name.c_str());

    // Then, for the content, create a child window that has the ImGuiWindowFlags_NoMove flag (if not moveable)
    ImGuiWindowFlags flags = _moveable ? 0 : ImGuiWindowFlags_NoMove;
    ImGui::BeginChild("Content", ImVec2(0, 0), false, flags);

    _imageMutex.lock();
    // Convert mat to texture
    ImTextureID texture = MatToTexture(_image);
    // Draw texture
    ImGui::Image(texture, ImVec2(_image.cols, _image.rows));
    _imageMutex.unlock();

    // Get the coords of the above image
    _windowPos = ImGui::GetItemRectMin();

    // End child and main window
    ImGui::EndChild();
    ImGui::End();
}

/**
 * Gets the mouse position relative to this window
 */
cv::Point2f ImageWidget::GetMousePos()
{
    ImVec2 mousePosVec2 = ImGui::GetMousePos();
    cv::Point2f mousePosAbsolute{mousePosVec2.x, mousePosVec2.y};
    cv::Point2f windowPos = cv::Point2f(_windowPos.x, _windowPos.y);
    return mousePosAbsolute - windowPos;
}

/**
 * Returns true if the mouse is over this window
 */
bool ImageWidget::IsMouseOver()
{
    return GetMousePos().inside(cv::Rect2f(0, 0, _image.cols, _image.rows));
}