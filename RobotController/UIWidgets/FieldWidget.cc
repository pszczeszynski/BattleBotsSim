#include "FieldWidget.h"
#include "../RobotController.h"
#include "../GuiUtils.h"
#include "../RobotConfig.h"
#include "../Input/InputState.h"

FieldWidget* FieldWidget::_instance = nullptr;

FieldWidget::FieldWidget() : ImageWidget("Field", RobotController::GetInstance().GetDrawingImage(), false)
{
    _instance = this;
}

void FieldWidget::AdjustFieldCrop()
{
    static int cornerToAdjust = -1;
    static cv::Point2f mousePosLast = cv::Point2f(0, 0);
    static cv::Point2f cornerHandles[4] = {cv::Point2f(0, 0),
                                           cv::Point2f(WIDTH, 0),
                                           cv::Point2f(WIDTH, HEIGHT),
                                           cv::Point2f(0, HEIGHT)};

    const double CORNER_DIST_THRESH = 20.0;

    // get the curr mouse position
    cv::Point2f currMousePos = GetMousePos();

    cv::Mat& drawingImage = RobotController::GetInstance().GetDrawingImage();

    // draw the corners
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

    // if the user isn't pressing shift and is over the image
    if (!InputState::GetInstance().IsKeyDown(ImGuiKey_LeftShift))
    {
        // corner adjustment

        // If the user left clicks near one of the corners
        if (InputState::GetInstance().IsMouseDown(0))
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
        else if (IsMouseOver())
        {
            // robot tracker calibration
            // if the user left clicks, aren't pressing shift, and are over the image, and not near a corner
            if (InputState::GetInstance().IsMouseDown(0))
            {
                // set the robot to the mouse position
                RobotOdometry::Robot().UpdateForceSetPosAndVel(currMousePos, cv::Point2f{0, 0});
            }

            // if the user right clicks
            if (InputState::GetInstance().IsMouseDown(1))// && input.IsMouseOverImage())
            {
                // set the opponent to the mouse position
                RobotOdometry::Opponent().UpdateForceSetPosAndVel(currMousePos, cv::Point2f{0, 0});
            }
        }
    }
    else if (IsMouseOver()) // else the user is pressing shift
    {
        // if the user presses the left mouse button with shift
        if (InputState::GetInstance().IsMouseDown(0))
        {
            // set the robot angle
            cv::Point2f robotPos = RobotOdometry::Robot().GetPosition();
            double newAngle = atan2(currMousePos.y - robotPos.y, currMousePos.x - robotPos.x);
            RobotOdometry::Robot().UpdateForceSetAngle(newAngle);
        }
    }

    // save the last mouse position
    mousePosLast = currMousePos;
}


FieldWidget* FieldWidget::GetInstance()
{
    return _instance;
}

