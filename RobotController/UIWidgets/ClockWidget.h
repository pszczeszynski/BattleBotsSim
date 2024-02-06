#pragma once
// similar to Clock, but includes a graph
#include "../Clock.h"

#include <vector>
#include <string>
#include "GraphWidget.h"


class ClockWidget : public Clock
{
public:
    ClockWidget(std::string label);
    ~ClockWidget();

    static void DrawAll();
    std::string getLabel() const { return _label; }

    virtual double markEnd() override;
private:
    static std::vector<ClockWidget*>& Instances();

    std::string _label;

    bool _showGraph = false;

    GraphWidget* _graph = nullptr;
};