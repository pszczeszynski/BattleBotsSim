#include "ClockWidget.h"
// imgui
#include "imgui.h"
#include <numeric>
#include <algorithm>

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

void ClockWidget::DrawAll()
{
    static ClockWidget loopClock("Total loop time");

    loopClock.markEnd();
    loopClock.markStart();

    std::vector<ClockWidget*>& instances = Instances();

    // create window called Profiling
    ImGui::Begin("Profiling");

    std::vector<double> averageTimes;
    std::vector<std::string> labels;

    // draw each clock
    for (ClockWidget* clock : instances)
    {
        // get the average time
        double averageTime = clock->getAverageTime();
        
        // add the average time and label
        averageTimes.push_back(averageTime);
        labels.push_back(clock->getLabel());
    }

    std::vector<int> indices(averageTimes.size());
    std::iota(indices.begin(), indices.end(), 0);

    // sort the indices based on the average times
    std::sort(indices.begin(), indices.end(), [&averageTimes](int i1, int i2)
              { return averageTimes[i1] > averageTimes[i2]; });
    
    double totalTime = loopClock.getAverageTime();
    // draw the bars and times
    for (int i : indices)
    {
        ImGui::Text("%s: %dms", labels[i].c_str(), (int) (averageTimes[i] * 1000.0));
        
        if (totalTime > 0.0)
        {
            ImGui::ProgressBar(averageTimes[i] / totalTime, ImVec2(0.0f, 0.0f));
        }
    }
}