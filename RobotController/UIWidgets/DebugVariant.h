#pragma once

#include <string>

enum class DebugVariant {
  Camera,
  Blob,
  Heuristic,
  Neural,
  Fusion,
  NeuralRot,
  Opencv,
  LKFlow,
};

const char* DebugVariantToString(DebugVariant v);
DebugVariant DebugVariantFromString(const std::string& s);
