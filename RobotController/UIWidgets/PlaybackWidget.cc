#include "PlaybackWidget.h"
#include <imgui.h>
#include <iostream>
#include "../RobotConfig.h"
#include "../PlaybackController.h"
#include "ColorScheme.h"

PlaybackWidget::PlaybackWidget()
{
#ifdef VIDEO_FILES
    std::filesystem::path dir("./Recordings");

    if(!std::filesystem::exists(dir)) { return; };
    
    // Get all the files in the directory
    for (const auto & entry : std::filesystem::directory_iterator("./Recordings"))
    {
        videoFiles.push_back(entry.path().string());
    }
#endif
}


/**
 * @brief Draws the playback interface
*/
void PlaybackWidget::Draw()
{
    PlaybackController& playback = PlaybackController::GetInstance();
    
    static int currentFile = 0; // Index of the currently selected file

    if (ImGui::Begin("Playback"))
    {
        if (ImGui::Button("Play"))
        {
            playback.Play();
        }

        ImGui::SameLine();
        if (ImGui::Button("Stop"))
        {
            playback.Stop();
        }

        ImGui::SameLine();

        bool popStyle = false;
        if (playback.IsPaused())
        {
            // Change the button color to green when isHighlighted is true
            ColorScheme::PushSuccessColors();
            popStyle = true;
        }

        if (ImGui::Button("Pause"))
        {
            playback.TogglePause();
        }
        
        if (popStyle)
        {
            ColorScheme::PopStatusColors(); 
            popStyle = false;
        }


        ImGui::SameLine();
        if (ImGui::Button("Restart"))
        {
            playback.Restart();
        }

        ImGui::SameLine();



        if (playback.IsReversing())
        {
            // Change the button color to green when isHighlighted is true
            ColorScheme::PushSuccessColors();
            popStyle = true;
        }

        if (ImGui::Button("Reverse"))
        {
            playback.ToggleReverse();
        }

        if (popStyle)
        {
            ColorScheme::PopStatusColors(); 
            popStyle = false;
        }

        ImGui::SameLine();
        float currentSpeed = playback.GetSpeed();
        ImGui::Text("    Speed:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        ImGui::InputFloat("##Speed", &currentSpeed);

        if (currentSpeed > 0.0f)
        {
            playback.SetSpeed(currentSpeed);
        }

        ImGui::SameLine();

        std::string selectedFile = "";

        if( videoFiles.size() > 0)
        {
            if (ImGui::BeginCombo("Files", videoFiles[currentFile].c_str()))
            {
                for (int n = 0; n < videoFiles.size(); n++)
                {
                    bool isSelected = (currentFile == n);
                    if (ImGui::Selectable(videoFiles[n].c_str(), isSelected))
                    {
                        currentFile = n;
                        selectedFile = videoFiles[n];                            
                    }    

                    if (isSelected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
        }

        // If the file is a video file, update the file to be read
        if ((selectedFile.length() > 3) &&  (selectedFile.find_last_of(".") > 0) && (selectedFile.substr(selectedFile.find_last_of(".")) == ".avi" || selectedFile.substr(selectedFile.find_last_of(".")) == ".mp4"))
        {
            std::cout << "Selected video file: " << selectedFile << std::endl;
            playback.SetFile(selectedFile);
        }

        // video scrub
        float videoPos = playback.GetVideoPosition();
        float videoLength = playback.GetVideoLength();
        if (ImGui::SliderFloat("Video Slider", &videoPos, 0.0f, videoLength))
        {
            playback.SetVideoPosition(videoPos);
        }

        // Allow preprocessing
        ImGui::Checkbox(":Preprocess Image", &PLAYBACK_PREPROCESS);
    }
    ImGui::End();
}

