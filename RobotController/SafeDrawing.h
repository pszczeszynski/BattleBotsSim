#pragma once

#include <opencv2/opencv.hpp>

void safe_circle(cv::Mat &img, cv::Point center, int radius, const cv::Scalar &color,
                 int thickness = 1, int lineType = cv::LINE_8, int shift = 0);

// Safe wrapper for safe_arrow
void safe_arrow(cv::Mat &img, cv::Point pt1, cv::Point pt2, const cv::Scalar &color,
                int thickness = 1, int lineType = cv::LINE_8, int shift = 0, double tipLength = 0.1);
