#pragma once

#include <string>

enum class DebugVariant : size_t {
  Camera = 0,
  Blob,
  Heuristic,
  Neural,
  Fusion,
  NeuralRot,
  Opencv,
  LKFlow,
  kCount
};

const char* DebugVariantToString(DebugVariant v);
DebugVariant DebugVariantFromString(const std::string& s);
