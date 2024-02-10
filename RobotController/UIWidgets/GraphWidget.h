#pragma once

#include <vector>
#include <string>
#include <deque>
#include <mutex>
class GraphWidget
{
public:
    GraphWidget(std::string title, float minYValue, float maxYValue, const char* unit, int historySize = 1000);
    ~GraphWidget();

    void AddData(float data);
    void SetData(const std::vector<float>& data);
    void SetData(const std::deque<float>& data);
    void ClearData();

    static void DrawAll();
private:
    static std::vector<GraphWidget*>& Instances();

    std::string _title;
    float _minYValue;
    float _maxYValue;
    const char* _unit;

    std::mutex _dataLock;
    std::deque<float> _data;

    const int _historySize;
};