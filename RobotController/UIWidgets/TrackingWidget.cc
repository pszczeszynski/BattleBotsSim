#include "TrackingWidget.h"
#include "../RobotController.h"
#include "../GuiUtils.h"
#include "../RobotConfig.h"
#include "../Input/InputState.h"
#include "CameraWidget.h"

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
}

void TrackingWidget::_GrabFrame()
{
    // 1. populate the tracking mat (for display purposes)
    static ICameraReceiver &camera = ICameraReceiver::GetInstance();
    if (camera.NewFrameReady(0))
    {
        long last_id = 0;
        long returnid = camera.GetFrame(_trackingMat, last_id);

        // If invalid frame exit
        if( (returnid < 0) || _trackingMat.empty() )
        {
            return;
        }

        // convert to rgb
        cv::cvtColor(_trackingMat, _trackingMat, cv::COLOR_GRAY2BGR);
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

    RobotOdometry &odometry = RobotController::GetInstance().odometry;
    BlobDetection &_odometry_Blob = odometry.GetBlobOdometry();
    HeuristicOdometry &_odometry_Heuristic = odometry.GetHeuristicOdometry();
    CVPosition& _odometry_Neural = odometry.GetNeuralOdometry();


    // go through every odometry algorithm and draw the tracking results
    if (_odometry_Blob.IsRunning())
    {
        OdometryData robot = _odometry_Blob.GetData(false);
        robot.Extrapolate(Clock::programClock.getElapsedTime());
        DrawX(_trackingMat, robot.robotPosition, blobColor, 20);

        OdometryData opponent = _odometry_Blob.GetData(true);
        opponent.Extrapolate(Clock::programClock.getElapsedTime());

        cv::circle(_trackingMat, opponent.robotPosition, 20, blobColor, 2);
    }

    if (_odometry_Heuristic.IsRunning())
    {
        OdometryData robot = _odometry_Heuristic.GetData(false);
        robot.Extrapolate(Clock::programClock.getElapsedTime());

        if (robot.robotPosValid)
        {
            DrawX(_trackingMat, robot.robotPosition, heuristicColor, 20);
        }


        OdometryData opponent = _odometry_Heuristic.GetData(true);
        opponent.Extrapolate(Clock::programClock.getElapsedTime());

        if (opponent.robotAngleValid)
        {
            cv::Point2f arrowEnd = opponent.robotPosition + cv::Point2f(50 * cos(opponent.robotAngle), 50 * sin(opponent.robotAngle));
            cv::arrowedLine(_trackingMat, opponent.robotPosition, arrowEnd, heuristicColor, 2);
        }

        if (opponent.robotPosValid)
        {
            cv::circle(_trackingMat, opponent.robotPosition, 20, heuristicColor, 2);
        }
    }

    if (_odometry_Neural.IsRunning())
    {
        OdometryData robot = _odometry_Neural.GetData(false);
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
                        _odometry_Heuristic.ForcePosition(GetMousePos(), false);
                    }
                    else
                    {
                        _odometry_Heuristic.SetPosition(GetMousePos(), false);
                    }

                    _odometry_Heuristic.SetVelocity(cv::Point2f(0,0), false);
                }
                else if (InputState::GetInstance().IsMouseDown(1))
                {
                    if (InputState::GetInstance().IsKeyDown(ImGuiKey_RightCtrl))
                    {
                        _odometry_Heuristic.ForcePosition(GetMousePos(), true);
                    }
                    else
                    {
                        _odometry_Heuristic.SetPosition(GetMousePos(), true);
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
    cv::arrowedLine(_trackingMat, robotPos, arrowEnd, cv::Scalar(255, 0, 0), 2);

    // draw opponent angle with arrow
    cv::Point2f opponentPos = odometry.Opponent().robotPosition;
    double opponentAngle = odometry.Opponent().robotAngle;
    arrowEnd = opponentPos + cv::Point2f(50 * cos(opponentAngle), 50 * sin(opponentAngle));
    cv::arrowedLine(_trackingMat, opponentPos, arrowEnd, cv::Scalar(0, 0, 255), 2);
}

cv::Mat& TrackingWidget::GetTrackingMat()
{
    return _trackingMat;
}

void TrackingWidget::Update()
{
    _GrabFrame();
    _AdjustFieldCrop();
    // Allow the user to mask out areas of the image.
    // They will be set to black when you get a frame from a camera
    _MaskOutRegions();

    _DrawAlgorithmData();

    UpdateMat(_trackingMat);
}