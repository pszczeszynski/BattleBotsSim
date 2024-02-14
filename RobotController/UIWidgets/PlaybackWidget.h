#pragma once
#include "imgui.h"
#include <vector>
#include <string>

class PlaybackWidget
{
public:
    static PlaybackWidget& GetInstance();
    PlaybackWidget();
    void Draw();

private:
    std::vector<std::string> videoFiles;

};