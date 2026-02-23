#include "DebugVariant.h"

const char* DebugVariantToString(DebugVariant v) {
  switch (v) {
    case DebugVariant::Camera:
      return "Camera";
    case DebugVariant::Blob:
      return "Blob";
    case DebugVariant::Heuristic:
      return "Heuristic";
    case DebugVariant::Neural:
      return "Neural";
    case DebugVariant::Fusion:
      return "Fusion";
    case DebugVariant::NeuralRot:
      return "NeuralRot";
    case DebugVariant::Opencv:
      return "Opencv";
    case DebugVariant::LKFlow:
      return "LKFlow";
    case DebugVariant::kCount:
      return "";
  }
  return "Camera";
}

DebugVariant DebugVariantFromString(const std::string& s) {
  if (s == "Blob") return DebugVariant::Blob;
  if (s == "Heuristic") return DebugVariant::Heuristic;
  if (s == "Neural") return DebugVariant::Neural;
  if (s == "Fusion") return DebugVariant::Fusion;
  if (s == "NeuralRot") return DebugVariant::NeuralRot;
  if (s == "Opencv") return DebugVariant::Opencv;
  if (s == "LKFlow") return DebugVariant::LKFlow;
  return DebugVariant::Camera;
}
