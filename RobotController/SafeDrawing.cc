#include "SafeDrawing.h"

void safe_circle(cv::Mat &img, cv::Point center, int radius, const cv::Scalar &color,
                 int thickness, int lineType, int shift)
{
    // Constants from OpenCV's drawing module
    const int MAX_THICKNESS = 255;
    const int XY_SHIFT = 16;

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
