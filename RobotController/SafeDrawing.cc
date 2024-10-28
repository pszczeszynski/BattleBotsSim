#include "SafeDrawing.h"

// Constants from OpenCV's drawing module
const int MAX_THICKNESS = 255;
const int XY_SHIFT = 16;


void safe_circle(cv::Mat &img, cv::Point center, int radius, const cv::Scalar &color,
                 int thickness, int lineType, int shift)
{
    // Check for valid radius
    if (radius < 0)
        return;

    // Check for valid thickness
    if (thickness > MAX_THICKNESS)
        return;

    // Check for valid shift
    if (shift < 0 || shift > XY_SHIFT)
        return;

    // Compute the scale factor based on shift
    double scale = 1.0 / (1 << shift);

    // Calculate the actual center coordinates and radius after scaling
    double cx = center.x * scale;
    double cy = center.y * scale;
    double r = radius * scale;

    // Compute the bounding rectangle of the circle
    double left = cx - r;
    double right = cx + r;
    double top = cy - r;
    double bottom = cy + r;

    // Check if the circle is completely outside the image bounds
    if (left >= img.cols || right < 0 || top >= img.rows || bottom < 0)
        return;

    // All checks passed; draw the circle
    cv::circle(img, center, radius, color, thickness, lineType, shift);
}

void safe_arrow(cv::Mat &img, cv::Point pt1, cv::Point pt2, const cv::Scalar &color,
                int thickness, int lineType, int shift, double tipLength)
{
    // Check for valid thickness
    if (thickness < 1 || thickness > MAX_THICKNESS)
        return;

    // Check for valid shift
    if (shift < 0 || shift > XY_SHIFT)
        return;

    // Check for valid tipLength
    if (tipLength < 0.0 || tipLength > 1.0)
        return;

    // Optionally, check if both points are within the image bounds
    // (Not strictly necessary, as OpenCV handles out-of-bounds points gracefully)
    if ((pt1.x < 0 && pt2.x < 0) || (pt1.x >= img.cols && pt2.x >= img.cols) ||
        (pt1.y < 0 && pt2.y < 0) || (pt1.y >= img.rows && pt2.y >= img.rows))
    {
        return;
    }

    // All checks passed; draw the arrowed line
    cv::arrowedLine(img, pt1, pt2, color, thickness, lineType, shift, tipLength);
}
