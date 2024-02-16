 #include "ClockWidget.h"
// imgui
#include "imgui.h"
#include <numeric>
#include <algorithm>
#include "imgui_internal.h"
#include <iostream>

std::vector<ClockWidget*>& ClockWidget::Instances()
{
    static std::vector<ClockWidget*> instances;
    return instances;
}

ClockWidget::ClockWidget(std::string label) : _label(label)
{
    Instances().push_back(this);
}

ClockWidget::~ClockWidget()
{
    std::vector<ClockWidget*>& instances = Instances();
    instances.erase(std::remove(instances.begin(), instances.end(), this), instances.end());
}

/**
 * Draws a progress bar with a target line
 * @param currentValue The current value
 * @param maxValue The maximum value (100%)
 * @param targetValue The target value (where the line should be)
 * @param size The size of the progress bar
*/
void DrawProgressBarWithTarget(float currentValue, float maxValue, float targetValue, const ImVec2 &size = ImVec2(-1, 0))
{
    // Calculate the percentages
    float currentPercentage = currentValue / maxValue;
    float targetPercentage = targetValue / maxValue;

    ImU32 barColor = IM_COL32(255, 225, 0, 255);

    // Draw the progress bar
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, barColor);
    ImGui::ProgressBar(currentPercentage, size, "");
    ImGui::PopStyleColor();

    // Calculate the position for the target line
    ImVec2 min = ImGui::GetItemRectMin();
    ImVec2 max = ImGui::GetItemRectMax();
    float targetLineX = min.x + (max.x - min.x) * targetPercentage;

    // Draw the target line
    ImGuiWindow *window = ImGui::GetCurrentWindow();
    window->DrawList->AddLine(ImVec2(targetLineX, min.y), ImVec2(targetLineX, max.y), IM_COL32(255, 0, 255, 255), 2.0f);
}

#define DEFAULT_TIME_SCALE 0.015

void ClockWidget::DrawAll()
{
    static float timeScale = DEFAULT_TIME_SCALE;

    std::vector<ClockWidget*>& instances = Instances();

    // create window called Profiling
    ImGui::Begin("Profiling");
    // center text
    ImGui::Text("Time Scale: %.2fms", timeScale * 1000.0);

    // Add an input box for the time scale
    ImGui::InputFloat("##", &timeScale, 0.001f, 0.01f, "%.3f");

    // space
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Spacing();

    std::vector<double> averageTimes;
    std::vector<double> maxTimes;
    std::vector<std::string> labels;

    // draw each clock
    for (ClockWidget* clock : instances)
    {
        // get the average time
        double averageTime = clock->getAverageTime();
        
        // add the average time and label
        averageTimes.push_back(averageTime);
        maxTimes.push_back(clock->getMaxTimeDifference());
        labels.push_back(clock->getLabel());
    }

    std::vector<int> indices(averageTimes.size());
    std::iota(indices.begin(), indices.end(), 0);

    // sort the indices based on the average times
    std::sort(indices.begin(), indices.end(), [&averageTimes](int i1, int i2)
              { return averageTimes[i1] > averageTimes[i2]; });
    

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 cursor_start = ImGui::GetCursorScreenPos();

    // draw the bars and times
    for (int i : indices)
    {
        ImGui::Text("%s: %.2fms", labels[i].c_str(), averageTimes[i] * 1000.0);

        // draw the progress bar with the target and max time
        double averageTime = averageTimes[i];
        double maxTime = maxTimes[i];
        DrawProgressBarWithTarget(averageTime, timeScale, maxTime, ImVec2(0, 0));

        // add text to right of progress bar with the max time
        ImGui::SameLine(0, 0);
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10);
        ImGui::Text("Max: %.2fms", maxTime * 1000.0);

        std::string showLabel = "Show##" + labels[i];
        std::string hideLabel = "Hide##" + labels[i];


        // draw a button to show the graph i
        ImGui::SameLine();
        if (ImGui::Button(showLabel.c_str()))
        {
            std::cout << "Drawing graph for " << instances[i]->getLabel() << std::endl;

            instances[i]->_showGraph = true;
        }
        ImGui::SameLine();
        if (ImGui::Button(hideLabel.c_str()))
        {
            std::cout << "Hiding graph for " << instances[i]->getLabel() << std::endl;

            instances[i]->_showGraph = false;
        }
    }

    // for each clock, draw the graph if it is shown
    for (ClockWidget* clock : instances)
    {
        if (clock->_showGraph)
        {
            if (clock->_graph == nullptr)
            {
                clock->_graph = new GraphWidget(clock->getLabel(), 0.0, timeScale * 1000, "ms", 1000);
            }
        }
        else
        {
            if (clock->_graph != nullptr)
            {
                delete clock->_graph;
                clock->_graph = nullptr;
            }
        }
    }

    ImGui::End();
}

double ClockWidget::markEnd()
{
    // call super method
    double timeDifference = Clock::markEnd();

    // check if have graph
    if (_graph != nullptr)
    {
        _graph->AddData(timeDifference * 1000.0);
    }

    return timeDifference;
}