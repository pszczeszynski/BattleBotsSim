#pragma once

#include <opencv2/opencv.hpp>

/**
 * Owns field-crop editing state for TrackingWidget.
 * Draws 4 draggable corner handles and updates preprocess_tl/tr/br/bl_*
 * directly.
 */
class TrackingFieldCropEditor {
 public:
  /**
   * Performs handle rendering and input handling.
   * Reads/writes preprocess_tl_x, preprocess_tl_y, etc. (RobotConfig globals).
   */
  void Update(cv::Point2f mousePos, bool mouseOverImage, cv::Mat& overlayMat);

 private:
  int _cornerToAdjust = -1;
  cv::Point2f _mousePosLast{0, 0};
  static constexpr double kCornerDistThresh = 20.0;
};
