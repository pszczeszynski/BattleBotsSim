#ifndef FIELD_H
#define FIELD_H

#include <vector>
#include <iostream>
#include <string>
#include <opencv2/core.hpp>

class Field {
public:
    // Constructor
    Field(int width, int height, float default_value = 0);

    // Getters
    int getWidth() const;
    int getHeight() const;

    // Methods to set values
    void setValue(int x, int y, float value);
    void setRegion(const std::vector<std::tuple<int, int, float>>& pointList);
    void resetRegion(const std::vector<std::tuple<int, int, float>>& pointList);

    // Method to get value at a position
    float getValue(int x, int y) const;

    // Display method
    void display(const std::vector<cv::Point>& pathPoints);

private:
    int width, height;
    float default_value;
    std::vector<std::vector<float>> field;
};

#endif // FIELD_H
