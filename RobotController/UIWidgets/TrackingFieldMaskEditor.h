#pragma once

#include <chrono>
#include <filesystem>
#include <functional>
#include <opencv2/opencv.hpp>

class InputState;

/**
 * Owns field-mask editing state and persistence for TrackingWidget.
 * Handles load/save of mask image, rectangle-drag masking (0 = visible, 255 = masked).
 */
class TrackingFieldMaskEditor {
 public:
  explicit TrackingFieldMaskEditor(std::filesystem::path maskPath);

  /** Load mask from file if it exists (convert to CV_8UC1, resize); else init mask to all visible (zeros). */
  void LoadOrInit(cv::Mat& mask, cv::Size expectedSize);

  /** Reset mask to all visible (zeros) with expectedSize and mark dirty. */
  void Clear(cv::Mat& mask, cv::Size expectedSize);

  /**
   * Run the mask-draw state machine. Only enters WAITING_FOR_CLICK when drawMaskMode is true.
   * On drag release: clip rect to mask bounds, fill with 255, set dirty.
   */
  void Update(bool drawMaskMode, bool isMouseOver, const InputState& input,
              std::function<cv::Point2f()> getMousePos, cv::Mat& mask);

  /** Write mask to disk when dirty; throttle to ~250ms. Clear dirty after successful write. */
  void SaveIfDirty(const cv::Mat& mask);

  /** True while the user is dragging a rectangle. */
  bool IsDragging() const;

  /** Current drag rectangle (valid when IsDragging()). */
  void GetDragRect(cv::Point2f& topLeft, cv::Point2f& bottomRight) const;

 private:
  enum class MaskState { LOCKED = 0, WAITING_FOR_CLICK, DRAGGING };

  std::filesystem::path _maskPath;
  MaskState _state = MaskState::LOCKED;
  cv::Point2f _topLeft{0, 0};
  cv::Point2f _bottomRight{0, 0};
  bool _dirty = false;
  std::chrono::steady_clock::time_point _lastSaveTime{};
  static constexpr std::chrono::milliseconds kSaveThrottleMs{250};
};
