#include "TrackingFieldCropEditor.h"

#include "../Globals.h"
#include "../Input/InputState.h"
#include "../RobotConfig.h"
#include "../SafeDrawing.h"
#include "CameraWidget.h"

void TrackingFieldCropEditor::Update(cv::Point2f mousePos, bool mouseOverImage,
                                     cv::Mat& overlayMat) {
  const bool enabledCrop = !CameraWidget::LockCamera;
  const bool shiftDown =
      InputState::GetInstance().IsKeyDown(ImGuiKey_LeftShift);
  const bool leftMouseDown = InputState::GetInstance().IsMouseDown(0);
  const cv::Size frameSize = overlayMat.empty()
                                ? cv::Size(WIDTH, HEIGHT)
                                : cv::Size(overlayMat.cols, overlayMat.rows);
  const int w = frameSize.width;
  const int h = frameSize.height;

  // Handles at the four corners of the frame
  const cv::Point2f cornerHandles[4] = {
      cv::Point2f(0, 0), cv::Point2f(static_cast<float>(w), 0),
      cv::Point2f(static_cast<float>(w), static_cast<float>(h)),
      cv::Point2f(0, static_cast<float>(h))};

  // Draw the corner handles
  if (enabledCrop && !overlayMat.empty()) {
    for (int i = 0; i < 4; i++) {
      if (cv::norm(cornerHandles[i] - mousePos) < kCornerDistThresh) {
        safe_circle(overlayMat, cornerHandles[i], kCornerDistThresh * 1.5,
                    cv::Scalar(255, 100, 255), 2);
      } else {
        safe_circle(overlayMat, cornerHandles[i], kCornerDistThresh,
                    cv::Scalar(255, 0, 255), 2);
      }
    }
  }

  // Corner selection/dragging only when: enabled, mouse over image, no shift, left mouse down
  const bool canAdjust =
      enabledCrop && mouseOverImage && !shiftDown && leftMouseDown;

  if (!shiftDown) {
    if (canAdjust) {
      if (_cornerToAdjust == -1) {
        for (int i = 0; i < 4; i++) {
          if (cv::norm(cornerHandles[i] - mousePos) < kCornerDistThresh) {
            _cornerToAdjust = i;
            break;
          }
        }
      }
    } else {
      _cornerToAdjust = -1;
    }

    cv::Point2f adjustment = _mousePosLast - mousePos;

    if (_cornerToAdjust == 0) {
      preprocess_tl_x += static_cast<int>(adjustment.x);
      preprocess_tl_y += static_cast<int>(adjustment.y);
    } else if (_cornerToAdjust == 1) {
      preprocess_tr_x += static_cast<int>(adjustment.x);
      preprocess_tr_y += static_cast<int>(adjustment.y);
    } else if (_cornerToAdjust == 2) {
      preprocess_br_x += static_cast<int>(adjustment.x);
      preprocess_br_y += static_cast<int>(adjustment.y);
    } else if (_cornerToAdjust == 3) {
      preprocess_bl_x += static_cast<int>(adjustment.x);
      preprocess_bl_y += static_cast<int>(adjustment.y);
    }
  }

  _mousePosLast = mousePos;
}
