#include "ValueBin.h"
#include "TrackingUtils.h"
#include <iostream>

ValueBin::ValueBin(int scale) : counts({}),
                                  valueSum({}),
                                  scaleSize(scale),
                                  numValues(0)
{
}

void ValueBin::AddValue(double value)
{
    double valueScaled = value * scaleSize;
    int rounded = round(valueScaled);

    if (counts.find(rounded) == counts.end())
    {
        counts[rounded] = 1;
        valueSum[rounded] = value;
    }
    else
    {
        counts[rounded]++;
        valueSum[rounded] += value;
    }

    numValues ++;
}

double ValueBin::GetAverageOfBin(double bin)
{
    bin *= scaleSize;
    int rounded = round(bin);
    if (counts.find(rounded) == counts.end())
    {
        return 0;
    }

    return valueSum[rounded] / counts[rounded];
}

double ValueBin::GetModeValue()
{
    if (counts.size() == 0)
    {
        return 0;
    }

    int maxCount = 0;
    int maxValue = 0;
    std::map<int, int>::iterator it;

    for (it = counts.begin(); it != counts.end(); it++)
    {
        if (it->second > maxCount)
        {
            maxCount = it->second;
            maxValue = it->first;
        }
    }

    return GetAverageOfBin(maxValue / scaleSize);
}

double ValueBin::GetAverageValue()
{
    double sum = 0;
    std::map<int, double>::iterator it;
    for (it = valueSum.begin(); it != valueSum.end(); it++)
    {
        sum += it->second;
    }

    return sum / numValues;
}

double ValueBin::GetWeightedAverageValue()
{
    double weightSum = 0;
    double sum = 0;

    std::map<int, double>::iterator it;
    for (it = valueSum.begin(); it != valueSum.end(); it++)
    {
        double thisBinMovement = GetAverageOfBin(((double) it->first) / scaleSize);
        double weight = fabs(thisBinMovement) * counts[it->first];
        weightSum += weight;
        sum += thisBinMovement * weight;
    }

    if (weightSum == 0)
    {
        return 0;
    }
    return sum / weightSum;
}

int ValueBin::GetCountForValue(double value)
{
    int rounded = round(value * scaleSize);
    if (counts.find(rounded) == counts.end())
    {
        return 0;
    }
    return counts[rounded];
}

int ValueBin::GetSize()
{
    int ret = 0;
    std::map<int, int>::iterator it;

    for (it = counts.begin(); it != counts.end(); it++)
    {
        ret += it->second;
    }

    return ret;
}
