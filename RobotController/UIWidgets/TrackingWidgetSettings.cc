#include "TrackingWidgetSettings.h"

#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "DebugVariant.h"

namespace {

std::vector<std::string> Split(const std::string& s, char delim) {
  std::vector<std::string> result;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    result.push_back(item);
  }
  return result;
}

}  // namespace

std::string SerializeTrackingWidgetSettings(const TrackingWidgetSettings& s) {
  std::stringstream ss;

  ss << "showCamera=" << (s.showCamera ? "1" : "0") << ";";
  ss << "showBlob=" << (s.showBlob ? "1" : "0") << ";";
  ss << "showHeuristic=" << (s.showHeuristic ? "1" : "0") << ";";
  ss << "showNeural=" << (s.showNeural ? "1" : "0") << ";";
  ss << "showNeuralRot=" << (s.showNeuralRot ? "1" : "0") << ";";
  ss << "showFusion=" << (s.showFusion ? "1" : "0") << ";";
  ss << "showOpencv=" << (s.showOpencv ? "1" : "0") << ";";
  ss << "showLKFlow=" << (s.showLKFlow ? "1" : "0") << ";";

  ss << "outputVideoFile=" << s.outputVideoFile << ";";

  ss << "variantColors=";
  for (size_t i = 0; i < kTrackingWidgetSettingsVariantCount; ++i) {
    const ImVec4& c = s.variantColors[i];
    ss << DebugVariantToString(static_cast<DebugVariant>(i)) << ":" << c.x
       << "," << c.y << "," << c.z << "," << c.w << "|";
  }
  ss << ";";

  ss << "variantOffsets=";
  for (size_t i = 0; i < kTrackingWidgetSettingsVariantCount; ++i) {
    const cv::Point& p = s.variantOffsets[i];
    ss << DebugVariantToString(static_cast<DebugVariant>(i)) << ":" << p.x
       << "," << p.y << "|";
  }
  ss << ";";

  return ss.str();
}

TrackingWidgetSettings DeserializeTrackingWidgetSettings(
    std::string_view text,
    const TrackingWidgetSettings& base) {
  TrackingWidgetSettings s = base;

  std::string content(text);
  std::istringstream ss(content);
  std::string token;

  while (std::getline(ss, token, ';')) {
    if (token.empty()) continue;
    auto keyValue = Split(token, '=');
    if (keyValue.size() != 2) continue;
    const std::string& key = keyValue[0];
    const std::string& value = keyValue[1];

    if (key == "showCamera")
      s.showCamera = (value == "1");
    else if (key == "showBlob")
      s.showBlob = (value == "1");
    else if (key == "showHeuristic")
      s.showHeuristic = (value == "1");
    else if (key == "showNeural")
      s.showNeural = (value == "1");
    else if (key == "showNeuralRot")
      s.showNeuralRot = (value == "1");
    else if (key == "showFusion")
      s.showFusion = (value == "1");
    else if (key == "showOpencv")
      s.showOpencv = (value == "1");
    else if (key == "showLKFlow")
      s.showLKFlow = (value == "1");

    else if (key == "outputVideoFile")
      s.outputVideoFile = value;

    else if (key == "variantColors") {
      auto colorEntries = Split(value, '|');
      for (const auto& entry : colorEntries) {
        if (entry.empty()) continue;
        auto colorData = Split(entry, ':');
        if (colorData.size() != 2) continue;
        auto colors = Split(colorData[1], ',');
        if (colors.size() != 4) continue;
        size_t i =
            static_cast<size_t>(DebugVariantFromString(colorData[0]));
        if (i < kTrackingWidgetSettingsVariantCount) {
          s.variantColors[i] =
              ImVec4(std::stof(colors[0]), std::stof(colors[1]),
                     std::stof(colors[2]), std::stof(colors[3]));
        }
      }
    }

    else if (key == "variantOffsets") {
      auto offsetEntries = Split(value, '|');
      for (const auto& entry : offsetEntries) {
        if (entry.empty()) continue;
        auto offsetData = Split(entry, ':');
        if (offsetData.size() != 2) continue;
        auto offsets = Split(offsetData[1], ',');
        if (offsets.size() != 2) continue;
        size_t i =
            static_cast<size_t>(DebugVariantFromString(offsetData[0]));
        if (i < kTrackingWidgetSettingsVariantCount) {
          s.variantOffsets[i] =
              cv::Point(std::stoi(offsets[0]), std::stoi(offsets[1]));
        }
      }
    }
  }

  return s;
}
