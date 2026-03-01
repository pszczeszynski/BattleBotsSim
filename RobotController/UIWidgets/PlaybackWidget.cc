#include "PlaybackWidget.h"
#include <imgui.h>
#include <iostream>
#include <filesystem>
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
        // Keyboard shortcuts for step (N/P) when window focused and paused
        // Edge-triggered: one press = one step (no repeat when holding)
        if (ImGui::IsWindowFocused() && playback.IsPaused())
        {
            if (ImGui::IsKeyPressed(ImGuiKey_N, false))
            {
                playback.RequestStepForward();
            }
            else if (ImGui::IsKeyPressed(ImGuiKey_P, false))
            {
                playback.RequestStepBackward();
            }
        }

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

        // Step forward/backward (only when paused)
        if (playback.IsPaused())
        {
            if (ImGui::Button("Step Fwd (N)"))
            {
                playback.RequestStepForward();
            }
            ImGui::SameLine();
            if (ImGui::Button("Step Back (P)"))
            {
                playback.RequestStepBackward();
            }
            ImGui::SameLine();
        }

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

        // video scrub (frame-based) - sync from playback only when not dragging to avoid bounce
        static int displayFrame = 0;
        static bool sliderWasActive = false;
        int64_t maxFrame = playback.GetFrameCount() - 1;
        int maxFrameInt = static_cast<int>(maxFrame > 0 ? maxFrame : 0);
        if (!sliderWasActive)
        {
            displayFrame = static_cast<int>(playback.GetFrame());
        }
        if (ImGui::SliderInt("Frame", &displayFrame, 0, maxFrameInt))
        {
            playback.RequestSeekFrame(static_cast<int64_t>(displayFrame));
        }
        sliderWasActive = ImGui::IsItemActive();

        // Allow preprocessing
        ImGui::Checkbox(":Preprocess Image", &PLAYBACK_PREPROCESS);
    }
    ImGui::End();
}

