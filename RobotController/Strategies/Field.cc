#include "Field.h"
#include <tuple>

// Constructor to initialize field dimensions and values
Field::Field(int width, int height, float default_value)
    : width(width), height(height), default_value(default_value) {
    // Initialize field with default value
    field.resize(height + 1, std::vector<float>(width + 1, default_value));
}

// Getter for width
int Field::getWidth() const {
    return width;
}

// Getter for height
int Field::getHeight() const {
    return height;
}

// Set a specific value at coordinates (x, y)
void Field::setValue(int x, int y, float value) {
    if (x >= 0 && x <= width && y >= 0 && y <= height) {
        field[y][x] = value;
    } else {
        //std::cout << "Error: Coordinates are out of bounds." << std::endl;
    }
}

// Set values for multiple points at once
void Field::setRegion(const std::vector<std::tuple<int, int, float>>& pointList) {
    for (const auto& point : pointList) {
        int x, y;
        float value;
        std::tie(x, y, value) = point;
        setValue(x, y, value);
    }
}

// set values for multiple points to 0
void Field::resetRegion(const std::vector<std::tuple<int, int, float>>& pointList) {
    for (const auto& point : pointList) {
        int x, y;
        float value;
        std::tie(x, y, value) = point;
        setValue(x, y, 0);
    }
}

// Get the value at coordinates (x, y)
float Field::getValue(int x, int y) const {
    if (x >= 0 && x <= width && y >= 0 && y <= height) {
        return field[y][x];
    } else {
        std::cout << "Error: Coordinates are out of bounds." << std::endl;
        return -1;  // Return a default error value
    }
}

// display in text form for debugging
void Field::display(const std::vector<cv::Point>& pathPoints) {
    system("cls");
    for (int y = height - 1; y >= 0; y--) {
        for (int x = 0; x < width; x++) {

            cv::Point testPoint(x, y);
            if(std::find(pathPoints.begin(), pathPoints.end(), testPoint) != pathPoints.end()) {
                std::cout << "P" << " ";
            }
            else if (field[y][x] == 0) {
                std::cout << "." << " ";
            }
            else {
                std::cout << field[y][x] << " ";
            }
        }
        std::cout << std::endl;
    }
    std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n";
}

