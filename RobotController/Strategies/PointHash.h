#include <opencv2/core.hpp>
#include <unordered_set>
#include <iostream>

// Custom hash function for cv::Point
struct PointHash {
    std::size_t operator()(const cv::Point& point) const {
        return std::hash<int>()(point.x) ^ (std::hash<int>()(point.y) << 1);
    }
};


