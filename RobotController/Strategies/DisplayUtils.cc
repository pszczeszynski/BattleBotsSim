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


// displays the lines from a list
void DisplayUtils::displayLines(std::vector<Line>& lines, cv::Scalar color) {
    for(int i = 0; i < lines.size(); i++) {
        std::pair<cv::Point2f, cv::Point2f> linePoints = lines[i].getLinePoints();
        cv::line(RobotController::GetInstance().GetDrawingImage(), linePoints.first, linePoints.second, color, 2);
    }
}


// displays a specific set of lines
void DisplayUtils::displayLinesIndices(std::vector<Line>& lines, std::vector<int> indices, cv::Scalar color) {
    for(int i = 0; i < indices.size(); i++) {

        if(indices[i] >= lines.size() || indices[i] < 0) { continue; } // no crashy

        cv::Point2f point1 = lines[indices[i]].getLinePoints().first;
        cv::Point2f point2 = lines[indices[i]].getLinePoints().second;
        cv::line(RobotController::GetInstance().GetDrawingImage(), point1, point2, color, 2);
    }
}


// WEWEWEWEWEEE
void DisplayUtils::emote() {

    cv::Mat drawing = RobotController::GetInstance().GetDrawingImage();

    cv::Mat logo = cv::imread("SkullEmote.png", cv::IMREAD_UNCHANGED); // keep alpha
    if (!logo.empty()) {
        // Pick top-left corner of where you want it
        cv::Point topLeft(100, 100);

        // Make sure it fits in the drawing surface
        cv::Rect roi(topLeft.x, topLeft.y, logo.cols, logo.rows);

        // If PNG has 4 channels (BGRA), split alpha and blend
        if (logo.channels() == 4) {
            std::vector<cv::Mat> layers;
            cv::split(logo, layers);
            cv::Mat bgr, alpha;
            cv::merge(std::vector<cv::Mat>{layers[0], layers[1], layers[2]}, bgr);
            alpha = layers[3];

            // blend PNG into drawing (simple alpha composite)
            cv::Mat region = drawing(roi);
            for (int y = 0; y < roi.height; ++y) {
                for (int x = 0; x < roi.width; ++x) {
                    float a = alpha.at<uchar>(y, x) / 255.0f;
                    for (int c = 0; c < 3; ++c) {
                        region.at<cv::Vec3b>(y, x)[c] =
                            static_cast<uchar>(a * bgr.at<cv::Vec3b>(y, x)[c] +
                                            (1.0f - a) * region.at<cv::Vec3b>(y, x)[c]);
                    }
                }
            }
        } else {
            // plain BGR copy if no alpha
            logo.copyTo(drawing(roi));
        }
    }

    // std::cout << "Working directory is: " << std::filesystem::current_path() << "\n";
}