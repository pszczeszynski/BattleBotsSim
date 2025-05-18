#include "RobotOdometry.h"
#include "MathUtils.h"
#include "../Common/Communication.h"
#include "RobotController.h"
#include "imgui.h"
#include "Input/InputState.h"
#include "RobotConfig.h"
#include "odometry/Neural/CVPosition.h"
#include "UIWidgets/GraphWidget.h"
#include "UIWidgets/ImageWidget.h"
#include "Globals.h"
#include "UIWidgets/ClockWidget.h"
#include "SafeDrawing.h"

std::mutex debugROStringForVideo_mutex;
std::string debugROStringForVideo = "";


RobotOdometry::RobotOdometry(ICameraReceiver &videoSource) : _videoSource(videoSource),
                                                             _odometry_Blob(&videoSource),
                                                             _odometry_Heuristic(&videoSource),
                                                             _odometry_Neural(&videoSource),
                                                             _odometry_Human(&videoSource, "11118", false),
                                                             _odometry_Human_Heuristic(&videoSource, "11119", true),
                                                             _odometry_NeuralRot(&videoSource),
                                                             _odometry_opencv(&videoSource)
{
}

RobotOdometry::~RobotOdometry()
{
    // If log file open, close it
    if (_logOdometryFileOpen)
    {
        _logOdometryFile.close();
        _logOdometryFileOpen = false;
    }
}

void RobotOdometry::_AdjustAngleWithArrowKeys()
{
    static Clock updateClock;

    float speed = 1.0;
    // if shift is held, multiply by 2
    if (InputState::GetInstance().IsKeyDown(ImGuiKey_LeftShift))
    {
        speed = 2.0;
    }

    Angle angleUserAdjust = Angle(updateClock.getElapsedTime() * 90 * M_PI / 180.0 * speed);
    updateClock.markStart();


    if (InputState::GetInstance().IsKeyDown(ImGuiKey_LeftArrow))
    {
        UpdateForceSetAngle(_dataRobot.robotAngle - angleUserAdjust, false);
    }
    else if (InputState::GetInstance().IsKeyDown(ImGuiKey_RightArrow))
    {
        UpdateForceSetAngle(_dataRobot.robotAngle + angleUserAdjust, false);
    }

    // opponent with up and 1 and 3
    if (InputState::GetInstance().IsKeyDown(ImGuiKey_1))
    {
        UpdateForceSetAngle(_dataOpponent.robotAngle - angleUserAdjust, true);
    }
    else if (InputState::GetInstance().IsKeyDown(ImGuiKey_3))
    {
        UpdateForceSetAngle(_dataOpponent.robotAngle + angleUserAdjust, true);
    }
}

// Updates internal Odometry data
void RobotOdometry::Update(int videoID)
{
    // ******************************
    // Retrieve new data if available
    bool newDataArrived = false;

    // Get Blob detection
    if (_odometry_Blob.IsRunning())
    {
        // Update our data
        if (_odometry_Blob.NewDataValid(_dataRobot_Blob.id, false))
        {
            _dataRobot_Blob = _odometry_Blob.GetData(false);
            newDataArrived = true;
        }

        // Update opponent data
        if (_odometry_Blob.NewDataValid(_dataOpponent_Blob.id, true))
        {
            _dataOpponent_Blob = _odometry_Blob.GetData(true);
            newDataArrived = true;
        }
    }

    if (_odometry_opencv.IsRunning())
    {
        // Update our data
        if (_odometry_opencv.NewDataValid(_dataRobot_opencv.id, false))
        {
            _dataRobot_opencv = _odometry_opencv.GetData(false);
            newDataArrived = true;
        }

        // Update opponent data
        if (_odometry_opencv.NewDataValid(_dataOpponent_opencv.id, true))
        {
            _dataOpponent_opencv = _odometry_opencv.GetData(true);
            newDataArrived = true;
        }
    }

    // Get Heuristic detection
    if (_odometry_Heuristic.IsRunning())
    {
        // Update our data
        if (_odometry_Heuristic.NewDataValid(_dataRobot_Heuristic.id, false))
        {
            _dataRobot_Heuristic = _odometry_Heuristic.GetData(false);
            newDataArrived = true;
        }

        // Update opponent data
        if (_odometry_Heuristic.NewDataValid(_dataOpponent_Heuristic.id, true))
        {
            _dataOpponent_Heuristic = _odometry_Heuristic.GetData(true);
            newDataArrived = true;
        }
    }

    // Get IMU
    if (_odometry_IMU.IsRunning())
    {
        // Update our data
        if (_odometry_IMU.NewDataValid(_dataRobot_IMU.id, false))
        {
            _dataRobot_IMU = _odometry_IMU.GetData(false);
            newDataArrived = true;
        }
    }

    // Get Neural
    if (_odometry_Neural.IsRunning())
    {
        // Update our data
        if (_odometry_Neural.NewDataValid(_dataRobot_Neural.id, false))
        {
            _dataRobot_Neural = _odometry_Neural.GetData(false);
            newDataArrived = true;
        }
    }

    // Get Neural Rot
    if (_odometry_NeuralRot.IsRunning())
    {
        // Update our data
        if (_odometry_NeuralRot.NewDataValid(_dataRobot_NeuralRot.id, false))
        {
            _dataRobot_NeuralRot = _odometry_NeuralRot.GetData(false);
            newDataArrived = true;
        }
    }

    if (_odometry_Human.IsRunning())
    {
        // Update our data
        if (_odometry_Human.NewDataValid(_dataRobot_Human.id, false))
        {
            _dataRobot_Human = _odometry_Human.GetData(false);
            _dataRobot_Human_is_new = true;
            newDataArrived = true;
        }
        else if (_odometry_Human.NewDataValid(_dataOpponent_Human.id, true))
        {
            _dataOpponent_Human = _odometry_Human.GetData(true);
            _dataOpponent_Human_is_new = true;
            newDataArrived = true;
        }
    }

    // No new data and thus nothing to do
    if (!newDataArrived)
    {
        return;
    }

    // At this time use only a priority set for all inputs
    std::unique_lock<std::mutex> locker(_updateMutex);
    FuseAndUpdatePositions(videoID);

    // // If IMU is running, then use IMU's angle information
    // if (_odometry_IMU.IsRunning() && _dataRobot_IMU.robotAngleValid)
    // {
    //     _dataRobot.robotAngleValid = true;
    //     _dataRobot.robotAngle = _dataRobot_IMU.robotAngle;
    //     _dataRobot.robotAngleVelocity = _dataRobot_IMU.robotAngleVelocity;
    // }

    // _dataOpponent.robotAngle = Angle(opponentRotationSim);



    // locker will get unlocked here automatically
}

void RobotOdometry::FuseAndUpdatePositions(int videoID)
{
    // Clear robot/opponent valid flags
    _dataRobot.robotPosValid = false;
    _dataOpponent.robotPosValid = false;
    _dataRobot.robotAngleValid = false;
    _dataOpponent.robotAngleValid = false;

    // ******************************
    // We have the following sources of data:
    // ALGORITH  | US POS |  US VEL | US ROT | US A.VEL | THEM POS | THEM VEL |  THEM ROT | THEM A.VEL
    // -------------------------------------------------------------------------------------------------
    // Heuristic |   X    |    X   |     X   |    X     |    X     |   X      |     X     |     X
    // Blob      |   X    |    X   |         |          |    X     |   X      |           |
    // Neural    |   X    |        |         |          |          |          |           |
    // NeuralRot |        |        |     X   |          |          |          |           |
    // IMU       |        |        |     X   |    X     |          |          |           |
    // Human     |<Hidden>|        |         |          | <Hidden> |          |     X     |    X
    //
    // TODO: Need to calculate Them A.Vel for human interface
    //
    // _odometry_Human: If angle changed, then update all angles
    //
    //
    // Priorities:
    //
    // GLOBAL PRECHECK:
    //      // If Neural says we should be swapped, then swap
    //      G1) if Neural = Heuristic.them: SWAP Heuristic
    //
    //      // If Neural agrees with blob, use Neural
    //      G2) if Neural != Heuristic.us && Neural=Blob.us: Heuristic.ForceUs(Neural)
    //
    //  US POS:
    //           Rule:  1) Heuristic, 2) Neural Pos, 3) Blob
    //           Post: If Heuristic.valid && Blob.pos-Heauristic.pos > threshold, setpos on blob
    //           If Heuristic.invalid, 1) setpos(blob) 2) setpos(neural)
    //
    //  US VEL:
    //           Rule: 1) Heuristic, 2) Blob
    //
    //  US ROT:
    //            Rule: 1) IMU (Neural is already fused), 2) Neural Rot 3) Heuristic
    //
    //  US A. VEL:
    //            Rule: 1) IMU, 2) Heuristic
    //
    //  THEM POS:
    //            Rule: 1) Heuristic, 2) Blob
    //            Post: If Heuristic.valid && Blob.pos-Heauristic.pos > threshold, setpos on blob
    //            If Heuristic.invalid, setpos(blob)
    //
    //  THEM VEL:
    //            Rule: 1) Heuristic, 2) Blob
    //
    //  THEM ROT:
    //            Rule: 1) Human, 2) Heuristic
    //
    //  THEM A. VEL:
    //            Rule: 1) Human, 2) Heuristic
    //
    //

    double currTime = Clock::programClock.getElapsedTime();
    dataRobot_Blob = _dataRobot_Blob;
    dataRobot_Heuristic = _dataRobot_Heuristic;
    dataRobot_Neural = _dataRobot_Neural;
    dataRobot_NeuralRot = _dataRobot_NeuralRot;
    dataRobot_IMU = _dataRobot_IMU;
    dataRobot_Human = _dataRobot_Human;

    dataOpponent_Blob = _dataOpponent_Blob;
    dataOpponent_Heuristic = _dataOpponent_Heuristic;
    dataOpponent_Human = _dataOpponent_Human;

    dataRobot_Blob.ExtrapolateBounded(currTime, MAX_EXTRAPOLATION_TIME_S);
    dataRobot_Heuristic.ExtrapolateBounded(currTime, MAX_EXTRAPOLATION_TIME_S);
    dataRobot_Neural.ExtrapolateBounded(currTime, MAX_EXTRAPOLATION_TIME_S);
    dataRobot_NeuralRot.ExtrapolateBounded(currTime, MAX_EXTRAPOLATION_TIME_S);
    dataRobot_NeuralRot.ExtrapolateBounded(currTime, MAX_EXTRAPOLATION_TIME_S);
    dataRobot_IMU.ExtrapolateBounded(currTime, MAX_EXTRAPOLATION_TIME_S);
    dataRobot_Human.ExtrapolateBounded(currTime, MAX_EXTRAPOLATION_TIME_S);

    dataOpponent_Blob.ExtrapolateBounded(currTime, MAX_EXTRAPOLATION_TIME_S);
    dataOpponent_Heuristic.ExtrapolateBounded(currTime, MAX_EXTRAPOLATION_TIME_S);


    // ******************************
    // HUMAN OVERRIDES
    // ******************************
    std::string debugROStringForVideo_tmp = "";

    bool humanThemAngle_valid = false;
    // Opponent rotation
    if (_odometry_Human.IsRunning() && _odometry_Heuristic.IsRunning() && dataOpponent_Human.robotAngleValid && _dataOpponent_Human_is_new)
    {
        // Clear flag
        _dataOpponent_Human_is_new = false;

        // Check if it arrived within reasonable time
        if (dataOpponent_Human.GetAge() < _dataAgeThreshold)
        {
            // Set angle for heuristic
            _odometry_Heuristic.SetAngle(dataOpponent_Human.robotAngle, true);
            _odometry_Heuristic.SetAngularVelocity(dataOpponent_Human.robotAngleVelocity, true);
            humanThemAngle_valid = true;
            debugROStringForVideo_tmp += "Human_Rot set = " + std::to_string(dataOpponent_Human.robotAngle) + "\n";
        }
    }

    // ******************************
    // HELPER VARIABLES
    // ******************************

    bool heuristicValid = _odometry_Heuristic.IsRunning() && (_dataRobot_Heuristic.GetAge() < _dataAgeThreshold);
    bool heuristicUsPos_valid = heuristicValid && _dataRobot_Heuristic.robotPosValid;
    bool heuristicUsAngle_valid = heuristicValid && _dataRobot_Heuristic.robotAngleValid;
    heuristicValid = _odometry_Heuristic.IsRunning() && (_dataOpponent_Heuristic.GetAge() < _dataAgeThreshold);
    bool heuristicThemPos_valid = heuristicValid && _dataOpponent_Heuristic.robotPosValid;
    bool heuristicThemAngle_valid = heuristicValid && _dataOpponent_Heuristic.robotAngleValid;

    bool blobValid = _odometry_Blob.IsRunning() && (_dataRobot_Blob.GetAge() < _dataAgeThreshold);
    bool blobUsPos_valid = blobValid && _dataRobot_Blob.robotPosValid;
    blobValid = _odometry_Blob.IsRunning() && (_dataOpponent_Blob.GetAge() < _dataAgeThreshold);
    bool blobThemPos_valid = blobValid && _dataOpponent_Blob.robotPosValid;

    bool neuralUsPos_valid = _odometry_Neural.IsRunning() && _dataRobot_Neural.robotPosValid && (_dataRobot_Neural.GetAge() < _dataAgeThreshold);

    bool neuralRot_valid = _odometry_NeuralRot.IsRunning() && _dataRobot_NeuralRot.robotAngleValid && (_dataRobot_NeuralRot.GetAge() < _dataAgeThreshold);

    bool imuValid = _odometry_IMU.IsRunning() && (_dataRobot_IMU.GetAge() < _dataAgeThreshold);
    bool imuUsRot_valid = imuValid && _dataRobot_IMU.robotAngleValid;

    debugROStringForVideo_tmp += "ALG U_P U_V U_R UAV T_P T_V T_R TAV\n";
    debugROStringForVideo_tmp += "Heu " + std::to_string(heuristicUsPos_valid) + "   " + std::to_string(heuristicUsPos_valid) + "   " + std::to_string(heuristicUsAngle_valid) + "   " + std::to_string(heuristicUsAngle_valid) + "   " + std::to_string(heuristicThemPos_valid) + "   " + std::to_string(heuristicThemPos_valid) + "   " + std::to_string(heuristicThemAngle_valid) + "\n";
    debugROStringForVideo_tmp += "Blb " + std::to_string(blobUsPos_valid)      + "   " + std::to_string(blobUsPos_valid)      + "   " +  std::string("   ")                    + "   " + std::string("   ")                     + "   " + std::to_string(blobThemPos_valid)      + "   " + std::to_string(blobThemPos_valid) + "\n";
    debugROStringForVideo_tmp += "Neu " + std::to_string(neuralUsPos_valid)    + "\n";
    debugROStringForVideo_tmp += "NeR " + std::string("   ")                   + "   " +    std::string("   ")                + "   " +  std::to_string(neuralRot_valid) + "\n";
    debugROStringForVideo_tmp += "IMU " + std::string("   ")                   + "   " +    std::string("   ")                + "   " +  std::to_string(imuUsRot_valid)  + "\n" ;

    // Log the odometry data to file
    LogOdometryToFile();


    // ******************************
    // Prechecks
    // ******************************

    // G1) if Neural = Heuristic.them: SWAP Heuristic
    if (neuralUsPos_valid && heuristicThemPos_valid)
    {
        // Check if point is inside bounding box
        if (dataOpponent_Heuristic.IsPointInside(dataRobot_Neural.robotPosition))
        {
            debugROStringForVideo_tmp += "G1: Swap Robots, Heu Both invalidated\n";

            // Swap heuristic
            _odometry_Heuristic.SwitchRobots();

            heuristicThemPos_valid = false;
            heuristicThemAngle_valid = false;
            heuristicUsPos_valid = false;
            heuristicUsAngle_valid = false;
        }
    }

    // G2) if Neural != Heuristic.us && Neural=Blob.us: Heuristic.ForceUs(Neural)
    if (neuralUsPos_valid && blobUsPos_valid && dataRobot_Blob.IsPointInside(dataRobot_Neural.robotPosition))
    {

        // Force position if heuristic doesn't agree
        if (!heuristicUsPos_valid || !dataRobot_Heuristic.IsPointInside(dataRobot_Neural.robotPosition))
        {
            debugROStringForVideo_tmp += "G2: Force position, Heu US invalidated\n";

            _odometry_Heuristic.ForcePosition(dataRobot_Neural.robotPosition, false);
            heuristicUsPos_valid = false;
            heuristicUsAngle_valid = false;
        }
    }

    // ******************************
    // PRIORITIES
    // ******************************
    //  US POS:
    //           Rule:  1) Heuristic, 2) Neural Pos, 3) Blob
    //           Post: If Heuristic.valid && Blob.pos-Heauristic.pos > threshold, setpos on blob
    //           If Heuristic.invalid, 1) setpos(blob) 2) setpos(neural)
    debugROStringForVideo_tmp += "US POS = ";

    if (heuristicUsPos_valid)
    {
        debugROStringForVideo_tmp += "Heu";
        _dataRobot.robotPosition = dataRobot_Heuristic.robotPosition;
        _dataRobot.time = dataRobot_Heuristic.time;
        _dataRobot.robotPosValid = true;

        // If blob position isn't inside our rectangle then set position
        if (blobUsPos_valid && !dataRobot_Heuristic.IsPointInside(dataRobot_Blob.robotPosition))
        {
            _odometry_Blob.SetPosition(dataRobot_Heuristic.robotPosition, false);
        }
    }
    else if (neuralUsPos_valid)
    {
        debugROStringForVideo_tmp += "Neu";
        _dataRobot.robotPosition = dataRobot_Neural.robotPosition;
        _dataRobot.time = dataRobot_Neural.time;
        _dataRobot.robotPosValid = true;
    }
    else if (blobUsPos_valid)
    {
        debugROStringForVideo_tmp += "Blob";
        _dataRobot.robotPosition = dataRobot_Blob.robotPosition;
        _dataRobot.time = dataRobot_Blob.time;
        _dataRobot.robotPosValid = true;
    }
    debugROStringForVideo_tmp += "\nUS Vel = ";

    //  US VEL:
    //           Rule: 1) Heuristic, 2) Blob
    if (heuristicUsPos_valid)
    {
        debugROStringForVideo_tmp += "Heu";
        _dataRobot.robotVelocity = dataRobot_Heuristic.robotVelocity;
    }
    else if (blobUsPos_valid)
    {
        if (neuralUsPos_valid)
        {
            debugROStringForVideo_tmp += "Neu";
            _dataRobot.robotVelocity = dataRobot_Neural.robotVelocity;
        }
        else
        {
            debugROStringForVideo_tmp += "Blob";
            _dataRobot.robotVelocity = dataRobot_Blob.robotVelocity;
        }
    }

    //  US ROT:
    //            Rule: 1) IMU (Neural is already fused), 2) Neural Rot 3) Heuristic
    debugROStringForVideo_tmp += "\nUS Rot = ";
    if (imuUsRot_valid)
    {
        debugROStringForVideo_tmp += "Imu";
        _dataRobot.robotAngle = dataRobot_IMU.robotAngle;
        _dataRobot.time_angle = dataRobot_IMU.time_angle;
        _dataRobot.robotAngleValid = true;

        // Set heuristor to IMU
        _odometry_Heuristic.SetAngle(_dataRobot.robotAngle, false);
    }
    else if (neuralRot_valid)
    {
        debugROStringForVideo_tmp += "NeuRot";
        _dataRobot.robotAngle = dataRobot_NeuralRot.robotAngle;
        _dataRobot.time_angle = dataRobot_NeuralRot.time_angle;
        _dataRobot.robotAngleValid = true;

        // Set heuristor to neural
        _odometry_Heuristic.SetAngle(_dataRobot.robotAngle, false);
    }
    else if (heuristicUsAngle_valid)
    {
        debugROStringForVideo_tmp += "Heu";
        _dataRobot.robotAngle = dataRobot_Heuristic.robotAngle;
        _dataRobot.time_angle = dataRobot_Heuristic.time_angle;
        _dataRobot.robotAngleValid = true;
    }

    debugROStringForVideo_tmp += "\nUS A_Vel = ";
    //  US A. VEL:
    //            Rule: 1) IMU, 2) Heuristic
    if (imuValid)
    {
        debugROStringForVideo_tmp += "Imu";
        _dataRobot.robotAngleVelocity = dataRobot_IMU.robotAngleVelocity;
    }
    else if (heuristicUsAngle_valid)
    {
        debugROStringForVideo_tmp += "Heu";
        _dataRobot.robotAngleVelocity = dataRobot_Heuristic.robotAngleVelocity;
    }

    //  THEM POS:
    //            Rule: 1) Heuristic, 2) Blob
    //            Post: If Heuristic.valid && Blob.pos-Heauristic.pos > threshold, setpos on blob
    //            If Heuristic.invalid, setpos(blob)
    debugROStringForVideo_tmp += "\nTHEM POS = ";
    if (heuristicThemPos_valid)
    {
        debugROStringForVideo_tmp += "Heu";
        _dataOpponent.robotPosition = dataOpponent_Heuristic.robotPosition;
        _dataOpponent.time = _dataRobot_Heuristic.time;
        _dataOpponent.robotPosValid = true;

        // If blob position isn't inside our rectangle then set position
        if (blobThemPos_valid && !_dataOpponent_Heuristic.IsPointInside(dataOpponent_Blob.robotPosition))
        {
            _odometry_Blob.SetPosition(dataOpponent_Heuristic.robotPosition, true);
        }
    }
    else if (blobThemPos_valid)
    {
        debugROStringForVideo_tmp += "Blob";
        _dataOpponent.robotPosition = dataOpponent_Blob.robotPosition;
        _dataOpponent.time = dataOpponent_Blob.time;
        _dataOpponent.robotPosValid = true;
        _odometry_Heuristic.SetPosition(dataOpponent_Blob.robotPosition, true);
    }

    debugROStringForVideo_tmp += "\nTHEM VEL = ";
    //  THEM VEL:
    //            Rule: 1) Heuristic, 2) Blob
    if (heuristicThemPos_valid)
    {
        debugROStringForVideo_tmp += "Heu";
        _dataOpponent.robotVelocity = dataOpponent_Heuristic.robotVelocity;
    }
    else if (blobThemPos_valid)
    {
        debugROStringForVideo_tmp += "Blob";
        _dataOpponent.robotVelocity = dataOpponent_Blob.robotVelocity;
    }

    debugROStringForVideo_tmp += "\nTHEM ROT = ";
    //  THEM ROT:
    //            Rule: 1) Human, 2) Heuristic
    if (humanThemAngle_valid)
    {
        debugROStringForVideo_tmp += "Human";
        _dataOpponent.robotAngle = dataOpponent_Human.robotAngle;
        _dataOpponent.time_angle = dataOpponent_Human.time_angle;
        _dataOpponent.robotAngleValid = true;
    }
    else if (heuristicThemAngle_valid)
    {
        debugROStringForVideo_tmp += "Heu";
        _dataOpponent.robotAngle = dataOpponent_Heuristic.robotAngle;
        _dataOpponent.time_angle = dataOpponent_Heuristic.time_angle;
        _dataOpponent.robotAngleValid = true;
    }

    debugROStringForVideo_tmp += "\nTHEM A_Vel = ";
    //  THEM A. VEL:
    //            Rule: 1) Human, 2) Heuristic
    if (humanThemAngle_valid)
    {
        debugROStringForVideo_tmp += "Human";
        _dataOpponent.robotAngleVelocity = dataOpponent_Human.robotAngleVelocity;
    }
    else if (heuristicThemAngle_valid)
    {
        debugROStringForVideo_tmp += "Heu";
        _dataOpponent.robotAngleVelocity = dataOpponent_Heuristic.robotAngleVelocity;
    }


#ifdef FORCE_SIM_DATA
    _dataRobot.robotPosValid = true;
    _dataRobot.robotPosition = robotPosSim;
    _dataRobot.robotVelocity = robotVelSim;
    _dataRobot.time = simReceiveLastTime;
    
    _dataOpponent.robotPosValid = true;
    _dataOpponent.robotPosition = opponentPosSim;
    _dataOpponent.robotVelocity = opponentVelSim;
    _dataOpponent.robotAngle = Angle(opponentRotationSim);
    _dataOpponent.robotAngleVelocity = opponentRotationVelSim;
    _dataOpponent.time = simReceiveLastTime;
#endif


    // ******************************
    //  FINISHED
    // ******************************
    TrackingWidget *trackingWidget = TrackingWidget::GetInstance();

    cv::Mat trackingMat;
    if (trackingWidget)
    {
        trackingMat = TrackingWidget::GetInstance()->GetTrackingMat();
    }

    if (!trackingMat.empty())
    {
        // put text at top center of the image
        int size = (MIN_ROBOT_BLOB_SIZE + MAX_ROBOT_BLOB_SIZE) / 4;

        // draw on the tracking mat a square
        cv::rectangle(trackingMat, _dataRobot.robotPosition - cv::Point2f(size, size), _dataRobot.robotPosition + cv::Point2f(size, size), cv::Scalar(255, 255, 255), 2);

        size = (MIN_OPPONENT_BLOB_SIZE + MAX_OPPONENT_BLOB_SIZE) / 4;

        // draw a circle for the opponent
        safe_circle(trackingMat, _dataOpponent.robotPosition, size, cv::Scalar(255, 255, 255), 2);
    }

    // Copy over to the shared debug string
    std::lock_guard<std::mutex> lock(debugROStringForVideo_mutex);
    debugROStringForVideo = debugROStringForVideo_tmp;
}

/*
 * Returns true if the tracking data can be trusted + used for orbit + kill mode
 * False otherwise.
 * 
 * Criteria are:
 * 1. Good if both blob + heuristic are valid + agree, and neural net isn't running
 * 2. Good if the current selected robot pos with the neural net + neural net valid + recent
 * 3. Bad otherwise
 */
bool RobotOdometry::IsTrackingGoodQuality()
{
    const double AGREEMENT_DIST_THRESH_PX = 50;

    bool heuristicValid = _odometry_Heuristic.IsRunning() && _dataRobot_Heuristic.robotPosValid && _dataRobot_Heuristic.GetAge() < 0.3;
    bool blobValid = _odometry_Blob.IsRunning(); // don't include valid, since it might just be stopped
    // the neural is valid if it isn't too old && it's running
    bool neuralValid = _odometry_Neural.IsRunning() && _dataRobot_Neural.robotPosValid && _dataRobot_Neural.GetAge() < 0.1;

    // std::cout << "heuristicValid: " << heuristicValid << std::endl;
    // std::cout << "blobValid: " << blobValid << std::endl;
    // std::cout << "neuralValid: " << neuralValid << std::endl;
    // std::cout << "heuristic robot pos valid? " << _dataRobot_Heuristic.robotPosValid << std::endl;
    // std::cout << "heuristic age: " << _dataRobot_Heuristic.GetAge() << std::endl;
    // if everybody is crying, return false :(
    if (!heuristicValid && !blobValid && !neuralValid)
    {
        // std::cout << "case 1" << std::endl;
        return false;
    }

    // if heuristic and blob valid, but neural net isn't running
    if (heuristicValid && blobValid && !neuralValid)
    {
        double distBetweenRobot = cv::norm(_dataRobot_Heuristic.robotPosition - _dataRobot_Blob.robotPosition);
        double distBetweenOpponent = cv::norm(_dataOpponent_Heuristic.robotPosition - _dataOpponent_Blob.robotPosition);

        // std::cout << "distBetweenRobot: " << distBetweenRobot << std::endl;
        // std::cout << "distBetweenOpponent: " << distBetweenOpponent << std::endl;
        // return true if they agree, false otherwise
        return distBetweenRobot < AGREEMENT_DIST_THRESH_PX && distBetweenOpponent < AGREEMENT_DIST_THRESH_PX;
    }

    // if the neural net is running and predicting
    if (neuralValid)
    {
        // check for agreement with neural net + agreement between the other two algorithms for the opponent
        double distToRobot = cv::norm(_dataRobot.robotPosition - _dataRobot_Neural.robotPosition);
        double distBetweenOpponent = cv::norm(_dataOpponent_Heuristic.robotPosition - _dataOpponent_Blob.robotPosition);
        // std::cout << "distToRobot: " << distToRobot << std::endl;
        // std::cout << "distBetweenOpponent: " << distBetweenOpponent << std::endl;

        return distToRobot < AGREEMENT_DIST_THRESH_PX && distBetweenOpponent < AGREEMENT_DIST_THRESH_PX;
    }

    // std::cout << "default case" << std::endl;
    // otherwise return false, since we don't have enough confidence
    return false;
}

/**
 * @brief returns the odometry data extrapolated to current time
 */
OdometryData RobotOdometry::Robot(double currTime)
{
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData currData = _dataRobot;
    locker.unlock();

    double extrapolateTime = min(currTime - currData.time, MAX_EXTRAPOLATION_TIME_S);
    currData.Extrapolate(extrapolateTime + currData.time);

    return currData;
}

/**
 * @brief Returns the angle to add to the global angle to get the internal imu angle (in radians)
*/
float RobotOdometry::GetIMUOffset()
{
    return _odometry_IMU.GetOffset();
}


/**
 * @brief returns the odometry data extrapolated to current time
 *
 */
OdometryData RobotOdometry::Opponent(double currTime)
{
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData currData = _dataOpponent;
    locker.unlock();

    double extrapolateTime = min(currTime - currData.time, MAX_EXTRAPOLATION_TIME_S);
    // currData.Extrapolate(extrapolateTime + currData.time);

    return currData;
}

static double GetImuAngleRad()
{
    // get angular velocity from imu
    return RobotController::GetInstance().GetIMUData().rotation;
}

static double GetImuAngleVelocityRadPerSec()
{
    // get angular velocity from imu
    return RobotController::GetInstance().GetIMUData().rotationVelocity;
}

/**
 * Allows artificially setting the angle of the robot
 * This is mainly used for manual recalibration.
 *
 * @return the new angle
 */
void RobotOdometry::UpdateForceSetAngle(double newAngle, bool opponentRobot)
{
    // Go through each Odometry and update it
    _odometry_Blob.SetAngle(newAngle, opponentRobot);
    _odometry_Heuristic.SetAngle(newAngle, opponentRobot);
    _odometry_IMU.SetAngle(newAngle, opponentRobot);

    // Update our own data
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData &odoData = (opponentRobot) ? _dataOpponent : _dataRobot;

    odoData.robotAngle = Angle(newAngle);
    odoData.robotAngleValid = true;
    odoData.robotAngleVelocity = 0;
}

/**
 * Allows artificially setting the position and velocity of the robot
 * This is mainly used for manual recalibration.
 *
 * @param newPos - the new position of the robot
 * @param newVel - the new velocity of the robot
 */
void RobotOdometry::UpdateForceSetPosAndVel(cv::Point2f newPos, cv::Point2f newVel, bool opponentRobot)
{
    // Go through each Odometry and update it
    _odometry_Blob.SetPosition(newPos, opponentRobot);
    _odometry_Blob.SetVelocity(newVel, opponentRobot);

    _odometry_Heuristic.SetPosition(newPos, opponentRobot);
    _odometry_Heuristic.SetVelocity(newVel, opponentRobot);

    _odometry_opencv.SetPosition(newPos, opponentRobot);

    // Update our own data
    std::unique_lock<std::mutex> locker(_updateMutex);
    OdometryData &odoData = (opponentRobot) ? _dataOpponent : _dataRobot;

    odoData.robotPosition = newPos;
    odoData.robotVelocity = newVel;
    odoData.robotPosValid = true;
}

// Switch position of robots
void RobotOdometry::SwitchRobots()
{
    // Switch all the Odometry
    _odometry_Blob.SwitchRobots();
    _odometry_Heuristic.SwitchRobots();

    // Update our own data
    std::unique_lock<std::mutex> locker(_updateMutex);

    OdometryData tempData = _dataRobot;
    _dataRobot = _dataOpponent;
    _dataOpponent = tempData;

    _dataRobot.isUs = true;
    _dataOpponent.isUs = false;
}

// Run Code
bool RobotOdometry::Run(OdometryAlg algorithm)
{
    switch (algorithm)
    {
    case OdometryAlg::Blob:
        return _odometry_Blob.Run();

    case OdometryAlg::Heuristic:
        return _odometry_Heuristic.Run();

    case OdometryAlg::IMU:
        return _odometry_IMU.Run();

    case OdometryAlg::Neural:
        return _odometry_Neural.Run();

    case OdometryAlg::Human:
        return _odometry_Human.Run() && _odometry_Human_Heuristic.Run();

    case OdometryAlg::NeuralRot:
        return _odometry_NeuralRot.Run();

    case OdometryAlg::OpenCV:
        return _odometry_opencv.Run();
    default:
        break;
    }

    return false;
}

// Stop Code
bool RobotOdometry::Stop(OdometryAlg algorithm)
{
    switch (algorithm)
    {
    case OdometryAlg::Blob:
        return _odometry_Blob.Stop();

    case OdometryAlg::Heuristic:
        return _odometry_Heuristic.Stop();

    case OdometryAlg::IMU:
        return _odometry_IMU.Stop();

    case OdometryAlg::Neural:
        return _odometry_Neural.Stop();
    
    case OdometryAlg::Human:
        return _odometry_Human.Stop() && _odometry_Human_Heuristic.Stop();
    
    case OdometryAlg::NeuralRot:
        return _odometry_NeuralRot.Stop();

    case OdometryAlg::OpenCV:
        return _odometry_opencv.Stop();
    }

    return false;
}

// IsRunning Code
bool RobotOdometry::IsRunning(OdometryAlg algorithm)
{
    switch (algorithm)
    {
    case OdometryAlg::Blob:
        return _odometry_Blob.IsRunning();

    case OdometryAlg::Heuristic:
        return _odometry_Heuristic.IsRunning();

    case OdometryAlg::IMU:
        return _odometry_IMU.IsRunning();

    case OdometryAlg::Neural:
        return _odometry_Neural.IsRunning();
    
    case OdometryAlg::Human:
        return _odometry_Human.IsRunning() && _odometry_Human_Heuristic.IsRunning();
    
    case OdometryAlg::NeuralRot:
        return _odometry_NeuralRot.IsRunning();
    
    case OdometryAlg::OpenCV:
        return _odometry_opencv.IsRunning();
    default:
        break;
    }

    return false;
}

HeuristicOdometry &RobotOdometry::GetHeuristicOdometry()
{
    return _odometry_Heuristic;
}

OdometryIMU &RobotOdometry::GetIMUOdometry()
{
    return _odometry_IMU;
}

CVPosition &RobotOdometry::GetNeuralOdometry()
{
    return _odometry_Neural;
}

BlobDetection &RobotOdometry::GetBlobOdometry()
{
    return _odometry_Blob;
}

CVRotation &RobotOdometry::GetNeuralRotOdometry()
{
    return _odometry_NeuralRot;
}

OpenCVTracker &RobotOdometry::GetOpenCVOdometry()
{
    return _odometry_opencv;
}


std::string getCurrentDateTime() {
    std::time_t now = std::time(nullptr);
    char time_str[20];
    std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    return std::string(time_str);
}

void RobotOdometry::LogOdometryToFile()
{
    if( !LOG_ODOMETRY_DATA  )
    {
        if( _logOdometryFile.is_open() )
        {
            // Close the file
            _logOdometryFileOpen = false;
            _logOdometryFile.close();
        }
        
        return;
    }


    if( LOG_ODOMETRY_DATA && !_logOdometryFileOpen )
    {
        // Open the file in append mode
        _logOdometryFileOpen = true;
        _logOdometryFile.open(_logOdometryFileName, std::ios::app);

        // Add header
        if( _logOdometryFile.is_open() )
        {
            _logOdometryFile << std::endl << "[" << getCurrentDateTime() << "]" << std::endl 
            << GetOdometryLog("UsHeu", dataRobot_Heuristic,true).str() << ","
            << GetOdometryLog("UsBlob", dataRobot_Blob,true).str() << ","
            << GetOdometryLog("UsNeural", dataRobot_Neural,true).str() << ","
            << GetOdometryLog("UsNeuralRot", dataRobot_NeuralRot,true).str() << ","
            << GetOdometryLog("UsIMU", dataRobot_IMU,true).str() << ","
            << GetOdometryLog("UsHuman", dataRobot_Human,true).str() << ","   
            << GetOdometryLog("ThemHeu", dataOpponent_Heuristic,true).str() << ","
            << GetOdometryLog("ThemBlob", dataOpponent_Blob,true).str() << ","
            << GetOdometryLog("ThemHuman", dataOpponent_Human,true).str()           
            << std::endl;
        }
    }




    // Add current Data to the file 
    if( _logOdometryFile.is_open() )
    {
     
        // Write the data to the file
        _logOdometryFile 
        << GetOdometryLog("UsHeu", dataRobot_Heuristic).str() << ","
        << GetOdometryLog("UsBlob", dataRobot_Blob).str() << ","
        << GetOdometryLog("UsNeural", dataRobot_Neural).str() << ","
        << GetOdometryLog("UsNeuralRot", dataRobot_NeuralRot).str() << ","
        << GetOdometryLog("UsIMU", dataRobot_IMU).str() << ","
        << GetOdometryLog("UsHuman", dataRobot_Human).str() << ","   
        << GetOdometryLog("ThemHeu", dataOpponent_Heuristic).str() << ","
        << GetOdometryLog("ThemBlob", dataOpponent_Blob).str() << ","
        << GetOdometryLog("ThemHuman", dataOpponent_Human).str()         
        << std::endl;
    }

}

std::stringstream RobotOdometry::GetOdometryLog( const std::string& name, OdometryData& odometry, bool doheader)
{
    std::stringstream ss;
    if( doheader )
    {
        ss << name << ",PosValid,AngleValid,frameID,frameTime,PosX,PosY,Angle,VelX,VelY,VelMag,VelAng,AVel";
    }
    else
    {

    ss << std::fixed << std::setprecision(2)
       << name << ","
       << ((odometry.robotPosValid) ? "1" : "0") << ","
       << ((odometry.robotAngleValid) ? "1" : "0") << ","
       << odometry.frameID << ","
       << odometry.time << ","
       << odometry.robotPosition.x << ","
       << odometry.robotPosition.y << ","
       << odometry.robotAngle << ","
       << odometry.robotVelocity.x << ","
       << odometry.robotVelocity.y << ","
       << std::sqrt(odometry.robotVelocity.x * odometry.robotVelocity.x + odometry.robotVelocity.y * odometry.robotVelocity.y) << ","
       << std::atan2(odometry.robotVelocity.y, odometry.robotVelocity.x) << ","
       << odometry.robotAngleVelocity;
    }

    return ss;
}


/**
 * @brief Forces the position of a tracking algorithm to be a certain value
 * 
 * @param alg - the algorithm to set the position of
 * @param pos - the position to set it to
 * @param opponent - whether to set the opponent or not
 */
void RobotOdometry::ForceSetPositionOfAlg(OdometryAlg alg, cv::Point2f pos, bool opponent)
{
    if (alg == OdometryAlg::Blob)
    {
        _odometry_Blob.SetPosition(pos, opponent);
    }
    else if (alg == OdometryAlg::Heuristic)
    {
        _odometry_Heuristic.SetPosition(pos, opponent);
    }
    else if (alg == OdometryAlg::IMU)
    {
        _odometry_IMU.SetPosition(pos, opponent);
    }
    else if (alg == OdometryAlg::Neural)
    {
        _odometry_Neural.SetPosition(pos, opponent);
    }
    else if (alg == OdometryAlg::OpenCV)
    {
        _odometry_opencv.SetPosition(pos, opponent);
    }
}


/**
 * @brief Forces the velocity of a tracking algorithm to be a certain value
 * 
 * @param alg - the algorithm to set the velocity of
 * @param vel - the velocity to set it to
 * @param opponent - whether to set the opponent or not
 */
void RobotOdometry::ForceSetVelocityOfAlg(OdometryAlg alg, cv::Point2f vel, bool opponent)
{
    if (alg == OdometryAlg::Blob)
    {
        _odometry_Blob.SetVelocity(vel, opponent);
    }
    else if (alg == OdometryAlg::Heuristic)
    {
        _odometry_Heuristic.SetVelocity(vel, opponent);
    }
    else if (alg == OdometryAlg::IMU)
    {
        std::cerr << "ERROR: Cannot set velocity for IMU" << std::endl;
    }
    else if (alg == OdometryAlg::Neural)
    {
        _odometry_Neural.SetVelocity(vel, opponent);
    }
    else if (alg == OdometryAlg::NeuralRot)
    {
        std::cerr << "ERROR: Cannot set velocity for NeuralRot" << std::endl;
    }
    else if (alg == OdometryAlg::OpenCV)
    {
        std::cerr << "ERROR: Cannot set velocity for opencv" << std::endl;
    }
}
