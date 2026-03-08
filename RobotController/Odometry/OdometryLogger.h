#pragma once

#include <fstream>
#include <mutex>
#include <string>

#include "OdometryData.h"

struct RawInputs;
struct FusionOutput;

class OdometryLogger {
 public:
  OdometryLogger();
  ~OdometryLogger();

  void LogFusion(const RawInputs& inputs, const FusionOutput& output,
                 double elapsedTime);

  // Formatted elapsed-time string for overlay on TrackingWidget.
  // Thread-safe; updated every Fuse() cycle.
  static std::string GetTimestampString();

 private:
  std::ofstream _file;
  bool _headerWritten = false;

  void _WriteHeader();
  static void _AppendOdometryHeader(std::ostream& os,
                                    const std::string& prefix);
  static void _AppendOdometryData(std::ostream& os, const OdometryData& data);

  static std::mutex _timestampMutex;
  static std::string _currentTimestamp;
};
