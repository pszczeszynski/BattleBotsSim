#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "DebugVariant.h"
#include "imgui.h"

/**
 * Lightweight input for a single layer in the tracking debug composition.
 */
struct VariantLayer {
  DebugVariant variant;
  const cv::Mat* image;
  bool visible;
  ImVec4 color;
};

/**
 * Composes multiple debug variant layers (grayscale or BGR) into a single BGR
 * output with tinting, alpha blending, and mask-from-nonzero semantics.
 * Scratch buffers are kept as members to avoid per-frame allocations.
 */
class FrameCompositor {
 public:
  FrameCompositor() = default;

  /**
   * Composes layers into outBgr (CV_8UC3). Output is cleared to black first.
   * Skips layers that are not visible, have null/empty image, or wrong size/type.
   * Handles CV_8UC1 (grayscale) and CV_8UC3 (BGR) inputs; alpha >= 0.99 uses
   * copyTo(mask) fast path, otherwise alpha blending.
   */
  void Compose(const std::vector<VariantLayer>& layers, cv::Size outputSize,
               cv::Mat& outBgr);

  /**
   * Ensures scratch buffers are allocated for the given size. Call when
   * outputSize changes to avoid reallocating inside Compose.
   */
  void ResetScratch(cv::Size outputSize);

 private:
  cv::Mat colorized_;
  cv::Mat mask_;
  cv::Mat temp3channel_;
  cv::Mat grayTemp_;
  cv::Mat blended_;
  cv::Mat maskedColorized_;
  cv::Mat inverseMask_;
};
