#ifndef SCOREREGION_H
#define SCOREREGION_H

#include <vector>
#include <iostream>
#include <cmath>
#include <opencv2/core.hpp>

class ScoreRegion {
public:
    // Constructor
    ScoreRegion(const std::vector<std::vector<float>>& region);

    // Get the value at the specified (x, y) coordinate in the grid
    float getValue(int x, int y) const;

    // Set the value at the specified (x, y) coordinate in the grid
    void setValue(int x, int y, float value);

    // Set the entire grid with multiple row lists
    void setGrid(const std::vector<std::vector<float>>& rows);

    // Returns global tile values knowing the position of the region
    std::vector<cv::Point> globalPoints(const cv::Point& position, const float& theta);

    // String representation of the grid for easy printing
    std::string toString() const;

private:
    int width;
    int height;
    std::vector<std::vector<float>> grid;
};

#endif // SCOREREGION_H
