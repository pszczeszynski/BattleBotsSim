#include "PlaybackWidget.h"
#include <iostream>
#include <filesystem>
#include "../RobotConfig.h"
#include "../Globals.h"

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
    
    static int currentFile = 0; // Index of the currently selected file

    if (ImGui::Begin("Playback"))
    {
        if (ImGui::Button("Play"))
        {
            playback_play = true;
            playback_pause = false;
        }

        ImGui::SameLine();
        if (ImGui::Button("Stop"))
        {
             playback_play = false;
        }

        ImGui::SameLine();

        bool popStyle = false;
        if (playback_pause)
        {
            // Change the button color to green when isHighlighted is true
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
            popStyle = true;
        }

        if (ImGui::Button("Pause"))
        {
            playback_pause = !playback_pause;
        }
        
        if (popStyle)
        {
            ImGui::PopStyleColor(); 
            popStyle = false;
        }


        ImGui::SameLine();
        if (ImGui::Button("Restart"))
        {
            playback_restart =true;
        }

        ImGui::SameLine();



        if (playback_goback)
        {
            // Change the button color to green when isHighlighted is true
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
            popStyle = true;
        }

        if (ImGui::Button("Reverse"))
        {
            playback_goback = !playback_goback;
        }

        if (popStyle)
        {
            ImGui::PopStyleColor(); 
            popStyle = false;
        }

        ImGui::SameLine();
        float prevSpeed = playback_speed;
        ImGui::Text("    Speed:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        ImGui::InputFloat("##Speed", &playback_speed);

        if( (playback_speed < 0) || (playback_speed == 0))
        {
            playback_speed = 1.0f;
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
            if( playback_file != selectedFile)
            {
                playback_file = selectedFile;
                playback_file_changed = true;
                playback_video_pos_s = 0;
                std::cout << "Selected video file: " << selectedFile << std::endl;

            }
            
        }

        // video scrub
        ImGui::SliderFloat("Video Slider", &playback_video_pos_s, 0.0f, playback_video_length_s);

        // Allow preprocessing
        ImGui::Checkbox(":Preprocess Image", &PLAYBACK_PREPROCESS);
    }
    ImGui::End();
}

