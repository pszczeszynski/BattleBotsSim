#include "TrackingFieldMaskEditor.h"

#include "../Input/InputState.h"

TrackingFieldMaskEditor::TrackingFieldMaskEditor(std::filesystem::path maskPath)
    : _maskPath(std::move(maskPath)) {}

void TrackingFieldMaskEditor::LoadOrInit(cv::Mat& mask, cv::Size expectedSize) {
  if (std::filesystem::exists(_maskPath)) {
    cv::Mat loaded = cv::imread(_maskPath.string());
    if (!loaded.empty()) {
      if (loaded.channels() == 1) {
        mask = loaded.clone();
      } else if (loaded.channels() == 3 || loaded.channels() == 4) {
        cv::cvtColor(
            loaded, mask,
            loaded.channels() == 3 ? cv::COLOR_BGR2GRAY : cv::COLOR_BGRA2GRAY);
      } else {
        mask = cv::Mat{expectedSize, CV_8UC1, cv::Scalar{0}};
      }
      if (mask.type() != CV_8UC1 || mask.cols != expectedSize.width ||
          mask.rows != expectedSize.height) {
        cv::Mat resized;
        cv::resize(mask, resized, expectedSize);
        mask = resized;
      }
      return;
    }
  }
  mask = cv::Mat{expectedSize, CV_8UC1, cv::Scalar{0}};
}

void TrackingFieldMaskEditor::Clear(cv::Mat& mask, cv::Size expectedSize) {
  mask = cv::Mat{expectedSize, CV_8UC1, cv::Scalar{0}};
  _dirty = true;
}

void TrackingFieldMaskEditor::Update(bool drawMaskMode, bool isMouseOver,
                                     const InputState& input,
                                     std::function<cv::Point2f()> getMousePos,
                                     cv::Mat& mask) {
  if (drawMaskMode) {
    if (_state == MaskState::LOCKED) {
      _state = MaskState::WAITING_FOR_CLICK;
    }
  } else {
    _state = MaskState::LOCKED;
  }

  if (_state == MaskState::WAITING_FOR_CLICK) {
    if (input.IsMouseDown(0) && isMouseOver) {
      _state = MaskState::DRAGGING;
      _topLeft = getMousePos();
    }
  }

  if (_state == MaskState::DRAGGING) {
    _bottomRight = getMousePos();

    if (!input.IsMouseDown(0)) {
      int x1 = static_cast<int>((std::min)(_topLeft.x, _bottomRight.x));
      int y1 = static_cast<int>((std::min)(_topLeft.y, _bottomRight.y));
      int x2 = static_cast<int>((std::max)(_topLeft.x, _bottomRight.x));
      int y2 = static_cast<int>((std::max)(_topLeft.y, _bottomRight.y));
      cv::Rect rect(x1, y1, x2 - x1, y2 - y1);
      cv::Rect maskBounds(0, 0, mask.cols, mask.rows);
      cv::Rect clipped(rect & maskBounds);

      if (clipped.width > 0 && clipped.height > 0) {
        cv::rectangle(mask, clipped, cv::Scalar(255), -1);
        _dirty = true;
      }

      _state = MaskState::WAITING_FOR_CLICK;
    }
  }
}

void TrackingFieldMaskEditor::SaveIfDirty(const cv::Mat& mask) {
  if (!_dirty) return;
  auto now = std::chrono::steady_clock::now();
  if (_lastSaveTime.time_since_epoch().count() != 0 &&
      (now - _lastSaveTime) < kSaveThrottleMs) {
    return;
  }
  if (cv::imwrite(_maskPath.string(), mask)) {
    _dirty = false;
    _lastSaveTime = now;
  }
}

bool TrackingFieldMaskEditor::IsDragging() const {
  return _state == MaskState::DRAGGING;
}

void TrackingFieldMaskEditor::GetDragRect(cv::Point2f& topLeft,
                                         cv::Point2f& bottomRight) const {
  topLeft = _topLeft;
  bottomRight = _bottomRight;
}
