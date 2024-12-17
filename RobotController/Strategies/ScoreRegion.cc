#include "ScoreRegion.h"
#include <cmath>
#include <string>
#include "../MathUtils.h"

// Constructor: Initializes the grid with the given region
ScoreRegion::ScoreRegion(const std::vector<std::vector<float>>& region)
    : width(region[0].size()), height(region.size()), grid(region) {}

// Get the value at (x, y) in the grid
float ScoreRegion::getValue(int x, int y) const {
    return grid[y][x];
}

// Set the value at (x, y) in the grid
void ScoreRegion::setValue(int x, int y, float value) {
    grid[y][x] = value;
}

// Set the entire grid using a list of rows
void ScoreRegion::setGrid(const std::vector<std::vector<float>>& rows) {
    if (rows.size() != height) {
        std::cerr << "Warning: The number of rows provided does not match the grid height (" << height << ")." << std::endl;
        return;
    }

    for (size_t row_index = 0; row_index < rows.size(); ++row_index) {
        if (rows[row_index].size() != width) {
            std::cerr << "Warning: The number of columns in row " << row_index << " does not match the grid width (" << width << ")." << std::endl;
            return;
        }
        grid[row_index] = rows[row_index];
    }
}

// Returns the global tiles knowing the position and orientation of the region
std::vector<cv::Point> ScoreRegion::globalPoints(const cv::Point& position, const float& theta) {
    std::vector<cv::Point> global_tiles;

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            float x_from_center = (-(width - 1) / 2 + x);
            float y_from_center = ((height - 1) / 2 - y);

            float exact_x = position.x + x_from_center * cos(theta - M_PI / 2) - y_from_center * sin(theta - M_PI / 2);
            float exact_y = position.y + x_from_center * sin(theta - M_PI / 2) + y_from_center * cos(theta - M_PI / 2);

            // if a point is denoted there, add it to the global tiles list
            if (grid[y][x] == 1.0f) {
                global_tiles.push_back(cv::Point(round(exact_x), round(exact_y)));
            }
        }
    }
    return global_tiles;
}

// String representation of the grid for easy printing
std::string ScoreRegion::toString() const {
    std::string result;
    for (const auto& row : grid) {
        for (const auto& cell : row) {
            result += std::to_string(cell) + " ";
        }
        result += "\n";
    }
    return result;
}
