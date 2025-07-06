#include "CameraWidget.h"
#include <iostream>
#include <filesystem>
#include "../RobotConfig.h"
#include "../Globals.h"
#include "FieldWidget.h"
#include "../RobotController.h"
#include "ConfigWidget.h"
#include "imgui.h"

#define ADD_IMVEC2(a, b) ImVec2((a).x + (b).x, (a).y + (b).y)

bool CameraWidget::LockCamera = true;
bool CameraWidget::ShowFisheyeImg = false;
bool CameraWidget::DoFisheye = true;
bool CameraWidget::tuningMode = false;
bool CameraWidget::DrawMask = false;

/**
 * @brief Draws the Camera Widget
*/
void CameraWidget::Draw()
{
    static float LAST_CAMERA_GAIN = CAMERA_GAIN;
    ImGui::Begin("Camera");

    ImGui::Checkbox("Lock Camera", &LockCamera);
    DoFisheye = FISHEYE_ENABLE;

    if (!LockCamera)
    {
        if (ImGui::Button("Reset Pre-Process Corners"))
        {
            preprocess_tl_y = 0;
            preprocess_tr_y = 0;
            preprocess_bl_y = HEIGHT;
            preprocess_br_y = HEIGHT;

            preprocess_tl_x = 0;
            preprocess_tr_x = WIDTH * 2;
            preprocess_bl_x = 0;
            preprocess_br_x = WIDTH * 2;
        }

        ImGui::Checkbox("Enable Fisheye", &FISHEYE_ENABLE);
        ImGui::SameLine();

        ImGui::Checkbox("Enable TUNING", &tuningMode);

        DoFisheye = FISHEYE_ENABLE;
        if (DoFisheye)
        {
            ImGui::Text("Fisheye Compensation:");
            ImGui::SliderFloat("Fisheye Scale", &FISHEYE_SCALE, 0, 20.0);
            ImGui::SliderFloat("Fisheye FL", &FISHEYE_FL, 50, 500.0);
            ImGui::SliderFloat("Fisheye Y", &FISHEYE_Y, 0, 1.0);
        }



        // masking
        ImGui::Checkbox("Draw Mask?", &DrawMask);
        if (ImGui::Button("Clear Mask"))
        {
            TrackingWidget::GetInstance()->ClearMask();
        }

        ImGui::SliderFloat("Camera GAIN", &CAMERA_GAIN, 0, 40.0);
        ImGui::SliderInt("Neural brightness adjust", &NEURAL_BRIGHTNESS_ADJUST, -40, 40);

        if (abs(LAST_CAMERA_GAIN - CAMERA_GAIN) > 0.001)
        {
            ICameraReceiver* camreceiver = &ICameraReceiver::GetInstance();
            if( (camreceiver != nullptr) && (camreceiver->GetType() == CameraType::REAL_CAMERA ) )
            {
                ((CameraReceiver*) camreceiver)->SetCameraGain(CAMERA_GAIN);
            }

            LAST_CAMERA_GAIN = CAMERA_GAIN;
            std::cout << "Camera GAIN: " << CAMERA_GAIN << std::endl;
        }

        if (ImGui::Button("Stop Video"))
        {
            RobotController::GetInstance().DumpVideo();
        }

        ImGui::NewLine();

        // Get all the points in

        ImGui::Text("Specify areas to compare. Size=");
        ImGui::SameLine();
        ImGui::PushItemWidth(80);
        ImGui::InputInt("##ptsize", &IMAGE_INTENSITY_SQR_size);
        ImGui::InputInt(":p1x   ", &IMAGE_INTENSITY_SQR_1_x);
        ImGui::SameLine();
        ImGui::InputInt(":p1y   ", &IMAGE_INTENSITY_SQR_1_y);
        ImGui::InputInt(":p2x   ", &IMAGE_INTENSITY_SQR_2_x);
        ImGui::SameLine();
        ImGui::InputInt(":p2y", &IMAGE_INTENSITY_SQR_2_y);
        ImGui::InputInt(":p3x   ", &IMAGE_INTENSITY_SQR_3_x);
        ImGui::SameLine();
        ImGui::InputInt(":p3y   ", &IMAGE_INTENSITY_SQR_3_y);
        ImGui::InputInt(":p4x   ", &IMAGE_INTENSITY_SQR_4_x);
        ImGui::SameLine();
        ImGui::InputInt(":p4y", &IMAGE_INTENSITY_SQR_4_y);

        static bool showAvgSqaures = false;
        static bool resetOverlay = true;
        ImGui::Checkbox("Show All Defined Sqaures", &showAvgSqaures);
                    
        cv::Mat& overlay = HeuristicOdometry::backgroundOverlay;    
 

        ImGui::PopItemWidth();        
        ImGui::PushItemWidth(80);
        ImGui::InputFloat("Avg Time Constant (Set < 0 to disable)", &IMAGE_INTENSITY_TIME_CONSTANT);
        

        // Heuristic settings
        ImGui::NewLine();
        ImGui::Text("Heuristic Settings:");
        ImGui::Text("Starting Box Left Side");

        ImGui::PushItemWidth(80);
        ImGui::InputInt(":LBOX TLx   ", &STARTING_LEFT_TL_x);
        ImGui::SameLine();
        ImGui::InputInt(":LBOX TLy   ", &STARTING_LEFT_TL_y);
        ImGui::InputInt(":LBOX BRx  ", &STARTING_LEFT_BR_x);
        ImGui::SameLine();
        ImGui::InputInt(":LBOX BRy", &STARTING_LEFT_BR_y);

        ImGui::InputInt(":RBOX TLx   ", &STARTING_RIGHT_TL_x);
        ImGui::SameLine();
        ImGui::InputInt(":RBOX TLy   ", &STARTING_RIGHT_TL_y);
        ImGui::InputInt(":RBOX BRx  ", &STARTING_RIGHT_BR_x);
        ImGui::SameLine();
        ImGui::InputInt(":RBOX BRy", &STARTING_RIGHT_BR_y);

        // DISPLAY ANY relevant boxes

        if(showAvgSqaures && HeuristicOdometry::bgOverlaySize.width > 0 && HeuristicOdometry::bgOverlaySize.height > 0)
        {            
            if( overlay.empty() || overlay.size() != HeuristicOdometry::bgOverlaySize)
            {
                overlay = cv::Mat::zeros(HeuristicOdometry::bgOverlaySize, CV_8UC3);
            }
            else{
                overlay.setTo(cv::Scalar(255));
            }
            
            cv::Scalar lineColor = cv::Scalar(255,0,255);
            cv::Size recsize(IMAGE_INTENSITY_SQR_size,IMAGE_INTENSITY_SQR_size);

            cv::Point p1 = cv::Point(IMAGE_INTENSITY_SQR_1_x, IMAGE_INTENSITY_SQR_1_y);
            cv::Point p2 = cv::Point(IMAGE_INTENSITY_SQR_2_x, IMAGE_INTENSITY_SQR_2_y);
            cv::Point p3 = cv::Point(IMAGE_INTENSITY_SQR_3_x, IMAGE_INTENSITY_SQR_3_y);
            cv::Point p4 = cv::Point(IMAGE_INTENSITY_SQR_4_x, IMAGE_INTENSITY_SQR_4_y);

            rectangle(overlay, cv::Rect(p1,recsize ), lineColor, 1,8);
            rectangle(overlay, cv::Rect(p2,recsize ), lineColor, 1,8);
            rectangle(overlay, cv::Rect(p3,recsize ), lineColor, 1,8);
            rectangle(overlay, cv::Rect(p4,recsize ), lineColor, 1,8);

            rectangle(overlay, cv::Rect(STARTING_LEFT_TL_x, STARTING_LEFT_TL_y, STARTING_LEFT_BR_x - STARTING_LEFT_TL_x, STARTING_LEFT_BR_y - STARTING_LEFT_TL_y), lineColor, 3, 8);
            rectangle(overlay, cv::Rect(STARTING_RIGHT_TL_x, STARTING_RIGHT_TL_y, STARTING_RIGHT_BR_x - STARTING_RIGHT_TL_x, STARTING_RIGHT_BR_y - STARTING_RIGHT_TL_y), lineColor, 3, 8);

            resetOverlay = true;

        }
        else if( resetOverlay)
        {
            overlay = cv::Mat::zeros(cv::Size(1,1), CV_8UC3);
            resetOverlay = false;
        }

        //
        //ImGui::Text(" Left=(%g,%g)", ConfigWidget::leftStart.x, ConfigWidget::leftStart.y);
        //ImGui::SameLine();
        //ImGui::Text("       Right=(%g,%g)", ConfigWidget::rightStart.x, ConfigWidget::rightStart.y);

        //    if (ImGui::Button("Set Left (L-Click)"))
        //    {
        //        ConfigWidget::leftStart = FieldWidget::leftClickPoint;
        //    }
        //    ImGui::SameLine();
        //    if (ImGui::Button("Set Right (R-Click)"))
        //    {
        //        ConfigWidget::rightStart = FieldWidget::rightClickPoint;
        //    }   
    
    }

    // ImGui::Checkbox("Show Fisheye Correction Img", &ShowFisheyeImg);
    ImGui::End();
}

