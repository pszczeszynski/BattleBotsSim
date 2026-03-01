#pragma once

#include <array>
#include <opencv2/core.hpp>
#include <string>
#include <string_view>

#include "DebugVariant.h"
#include "imgui.h"

constexpr size_t kTrackingWidgetSettingsVariantCount =
    static_cast<size_t>(DebugVariant::kCount);

struct TrackingWidgetSettings {
  bool showCamera = true;
  bool showBlob = false;
  bool showHeuristic = false;
  bool showNeural = false;
  bool showNeuralRot = false;
  bool showFusion = false;
  bool showOpencv = false;
  bool showLKFlow = false;

  std::string outputVideoFile;

  std::array<ImVec4, kTrackingWidgetSettingsVariantCount> variantColors{};
  std::array<cv::Point, kTrackingWidgetSettingsVariantCount> variantOffsets{};
};

std::string SerializeTrackingWidgetSettings(const TrackingWidgetSettings& s);

TrackingWidgetSettings DeserializeTrackingWidgetSettings(
    std::string_view text,
    const TrackingWidgetSettings& base);
