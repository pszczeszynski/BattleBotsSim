#include "CameraWidget.h"
#include <iostream>
#include <filesystem>
#include "../RobotConfig.h"
#include "../Globals.h"

bool CameraWidget::LockCamera = true;
bool CameraWidget::ShowFisheyeImg = false;
bool CameraWidget::DoFisheye = true;
bool CameraWidget::tuningMode = false;

/**
 * @brief Draws the Camera Widget
*/
void CameraWidget::Draw()
{
    ImGui::Begin("Camera");

    ImGui::Checkbox("Lock Camera", &LockCamera);
    DoFisheye = FISHEYE_ENABLE;

    if( !LockCamera) {    
        if( ImGui::Button("Reset Pre-Process Corners") )
        {
            preprocess_tl_y = 0;
            preprocess_tr_y = 0;
            preprocess_bl_y = HEIGHT;
            preprocess_br_y = HEIGHT;

            preprocess_tl_x = 0;
            preprocess_tr_x = WIDTH*2;
            preprocess_bl_x = 0;
            preprocess_br_x = WIDTH*2;
            
        }

        ImGui::Checkbox("Enable Fisheye", &FISHEYE_ENABLE);
        ImGui::SameLine();

        ImGui::Checkbox("Enable TUNING", &tuningMode);
        
        DoFisheye = FISHEYE_ENABLE;
        if( DoFisheye)
        {
            ImGui::Text("Fisheye Compensation:");
            ImGui::SliderFloat("Fisheye Scale", &FISHEYE_SCALE, 0, 20.0);
            ImGui::SliderFloat("Fisheye FL", &FISHEYE_FL, 50, 500.0);
            ImGui::SliderFloat("Fisheye Y", &FISHEYE_Y, 0, 1.0);
        }
    }

    // ImGui::Checkbox("Show Fisheye Correction Img", &ShowFisheyeImg);
    ImGui::End();
}
