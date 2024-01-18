// similar to Clock, but includes a graph
#include "../Clock.h"

#include <vector>
#include <string>


class ClockWidget : public Clock
{
public:
    ClockWidget(std::string label);
    ~ClockWidget();

    static void DrawAll();
    std::string getLabel() const { return _label; }
private:
    static std::vector<ClockWidget*>& Instances();
    std::string _label;
};