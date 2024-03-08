#include "FieldWidget.h"
#include "../RobotController.h"
#include "../GuiUtils.h"
#include "../RobotConfig.h"
#include "../Input/InputState.h"
#include "CameraWidget.h"

FieldWidget *FieldWidget::_instance = nullptr;
cv::Point2f FieldWidget::leftClickPoint = cv::Point2f(0, 0);
cv::Point2f FieldWidget::rightClickPoint = cv::Point2f(0, 0);

#define MASK_PATH "./backgrounds/fieldMask.jpg"

FieldWidget::FieldWidget() : ImageWidget("Field", RobotController::GetInstance().GetDrawingImage(), false),
    // initialize the mask to all white
    _fieldMask{WIDTH, HEIGHT, CV_8UC1, cv::Scalar{0}}
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
}

void FieldWidget::AdjustFieldCrop()
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

    cv::Mat &drawingImage = RobotController::GetInstance().GetDrawingImage();

    // draw the corners
    if (enableCrop)
    {
        for (int i = 0; i < 4; i++)
        {
            if (cv::norm(cornerHandles[i] - currMousePos) < CORNER_DIST_THRESH)
            {
                cv::circle(drawingImage, cornerHandles[i], CORNER_DIST_THRESH * 1.5, cv::Scalar(255, 100, 255), 2);
            }
            else
            {
                cv::circle(drawingImage, cornerHandles[i], CORNER_DIST_THRESH, cv::Scalar(255, 0, 255), 2);
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

FieldWidget *FieldWidget::GetInstance()
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
void FieldWidget::MaskOutRegions()
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
void FieldWidget::ClearMask()
{
    // set the mask to all white
    _fieldMask = cv::Mat{WIDTH, HEIGHT, CV_8UC1, cv::Scalar{0}};
    cv::imwrite(MASK_PATH, _fieldMask);
}

cv::Mat& FieldWidget::GetMask()
{
    return _fieldMask;
}