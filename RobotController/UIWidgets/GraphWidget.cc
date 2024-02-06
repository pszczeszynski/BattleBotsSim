#include "GraphWidget.h"

#include <imgui.h>

#include <vector>
#include <algorithm>

std::vector<GraphWidget *> &GraphWidget::Instances()
{
    static std::vector<GraphWidget *> instances;
    return instances;
}

GraphWidget::GraphWidget(std::string title, float minYValue, float maxYValue, const char *unit, int historySize)
    : _title(title), _minYValue(minYValue), _maxYValue(maxYValue), _unit(unit), _historySize(historySize)
{
    Instances().push_back(this);
}

GraphWidget::~GraphWidget()
{
    std::vector<GraphWidget *> &instances = Instances();
    instances.erase(std::remove(instances.begin(), instances.end(), this), instances.end());
}

/**
 * \brief
 * Draws a graph with the given data
 * 
*/
void DrawGraph(const char *title, const std::deque<float> &data, float minYValue, float maxYValue,
               const char *unit, const ImVec2 &graphSize = ImVec2(0, 150.0f),
               const ImVec4 &graphColor = ImVec4(0.2f, 0.6f, 0.86f, 1.0f)) // Default graph color
{
    const int numLabels = 3;

    // Center and display the title
    float windowWidth = ImGui::GetWindowWidth();
    float textWidth = ImGui::CalcTextSize(title).x;
    ImGui::SetCursorPosX((windowWidth - textWidth) / 2);
    ImGui::Text(title);

    ImVec2 graphPos = ImGui::GetCursorScreenPos();
    float graphHeight = graphSize.y;
    float range = maxYValue - minYValue;  // Total range of the data
    float step = range / (numLabels - 1); // Step value for each label

    // Draw y-axis labels
    for (int i = 0; i < numLabels; i++)
    {
        char label[32];
        // Adjust label to include the min value in the calculation
        snprintf(label, sizeof(label), "%.0f%s", minYValue + i * step, unit);

        float yPosition;
        if (i == 0)
        { // Bottommost label
            yPosition = graphPos.y + graphHeight - ImGui::GetTextLineHeight();
        }
        else if (i == numLabels - 1)
        { // Topmost label
            yPosition = graphPos.y;
        }
        else
        { // Middle label
            yPosition = graphPos.y + graphHeight - i * (graphHeight / (numLabels - 1)) - ImGui::GetTextLineHeight() * 0.5;
        }

        ImGui::SetCursorScreenPos(ImVec2(graphPos.x, yPosition));
        ImGui::TextUnformatted(label);
    }

    // Move cursor back to original position before drawing graph
    ImGui::SetCursorScreenPos(graphPos);
    float xOffset = 45.0f;
    ImGui::SameLine(xOffset);

    float graphWidth = ImGui::GetContentRegionAvail().x - xOffset;

    // Set the color for the graph line
    ImGui::PushStyleColor(ImGuiCol_PlotLines, graphColor);
    
    std::vector<float> contiguousData(data.begin(), data.end());

    // Display the graph, adjusting for the minimum value
    ImGui::PlotLines("", contiguousData.data(), data.size(), 0, nullptr, minYValue, maxYValue, ImVec2(graphWidth, graphHeight));

    // Pop the style color to revert to the previous setting
    ImGui::PopStyleColor();
}

void GraphWidget::AddData(float data)
{
    _data.push_back(data);

    if (_data.size() > _historySize)
    {
        _data.pop_front();
    }
}

void GraphWidget::SetData(const std::vector<float> &data)
{
    _data.assign(data.begin(), data.end());

    // if the data is larger than the history size, resize it (this takes the
    // last _historySize elements of the data vector)
    if (_data.size() > _historySize)
    {
        _data.resize(_historySize);
    }
}

void GraphWidget::SetData(const std::deque<float> &data)
{
    _data = data;

    // if the data is larger than the history size, resize it (this takes the
    // last _historySize elements of the data vector)
    if (_data.size() > _historySize)
    {
        _data.resize(_historySize);
    }
}

void GraphWidget::ClearData()
{
    _data.clear();
}

void GraphWidget::DrawAll()
{
    // create new window called Graphs
    ImGui::Begin("Graphs");

    // Draw all the graphs
    for (GraphWidget *widget : Instances())
    {
        DrawGraph(widget->_title.c_str(), widget->_data, widget->_minYValue, widget->_maxYValue, widget->_unit);
    }

    ImGui::End();
}