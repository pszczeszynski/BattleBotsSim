#include "DisplayUtils.h"
#include "../RobotController.h"


// display a list of points and interpolate colors
void DisplayUtils::displayPoints(std::vector<cv::Point2f>& points, cv::Scalar startColor, cv::Scalar endColor, int radius) {

    for (int i = 0; i < points.size(); i++) {

        float percent = (float) i / (points.size() - 1);
        cv::Scalar color = (endColor - startColor)*percent + startColor;

        safe_circle(RobotController::GetInstance().GetDrawingImage(), points[i], radius, color, radius);
    }
}


// display a list of points as connected lines
void DisplayUtils::displayPath(std::vector<cv::Point2f>& pathPoints, cv::Scalar startColor, cv::Scalar endColor, int thick) {

    if(pathPoints.size() < 1) { return; } // no crashy

    for (int i = 0; i < pathPoints.size() - 1; i++) {

        float percent = (float) i / (pathPoints.size() - 2);
        cv::Scalar color = (endColor - startColor)*percent + startColor;

        cv::line(RobotController::GetInstance().GetDrawingImage(), pathPoints[i], pathPoints[i + 1], color, thick);
    }
}