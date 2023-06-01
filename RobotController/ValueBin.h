
#ifndef VALUEBIN_H
#define VALUEBIN_H

#include <map>

class ValueBin
{
private:
    std::map<int, int> counts;
    std::map<int, double> valueSum;
    int scaleSize;
    int numValues;

public:
    ValueBin(int scale = 1);

    void AddValue(double);
    double GetAverageOfBin(double);
    double GetModeValue();
    double GetAverageValue();
    double GetWeightedAverageValue();
    int GetCountForValue(double);

    int GetSize();
};

#endif
