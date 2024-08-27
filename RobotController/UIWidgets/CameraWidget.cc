#include "CameraWidget.h"
#include <iostream>
#include <filesystem>
#include "../RobotConfig.h"
#include "../Globals.h"
#include "FieldWidget.h"
#include "../RobotController.h"

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

        if (abs(LAST_CAMERA_GAIN - CAMERA_GAIN) > 0.001)
        {
            ((CameraReceiver*) &ICameraReceiver::GetInstance())->SetCameraGain(CAMERA_GAIN);
            LAST_CAMERA_GAIN = CAMERA_GAIN;
            std::cout << "Camera GAIN: " << CAMERA_GAIN << std::endl;
        }

        if (ImGui::Button("Stop Video"))
        {
            RobotController::GetInstance().DumpVideo();
        }
    }

    // ImGui::Checkbox("Show Fisheye Correction Img", &ShowFisheyeImg);
    ImGui::End();
}

