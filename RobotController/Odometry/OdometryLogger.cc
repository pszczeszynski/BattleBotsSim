#include "OdometryLogger.h"

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "../OdometryPoller.h"
#include "../RobotOdometry.h"

std::mutex OdometryLogger::_timestampMutex;
std::string OdometryLogger::_currentTimestamp = "T+0.000";

OdometryLogger::OdometryLogger() {
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  auto now_tm = std::localtime(&now_time_t);
  std::stringstream ss;
  ss << std::put_time(now_tm, "%Y_%m_%d_%H_%M_%S");

  std::filesystem::create_directories("Logs");
  std::string filename = "Logs/odometry_fusion_" + ss.str() + ".csv";
  _file.open(filename);
  if (!_file.is_open()) {
    std::cerr << "OdometryLogger: failed to open " << filename << std::endl;
  }
}

OdometryLogger::~OdometryLogger() {
  if (_file.is_open()) {
    _file.close();
  }
}

// Per-algorithm columns: pos_valid, pos_x, pos_y, vel_x, vel_y, pos_time,
// angle_valid, angle_deg, angle_vel, angle_time, frame_id, pos_alg, angle_alg
void OdometryLogger::_AppendOdometryHeader(std::ostream& os,
                                           const std::string& p) {
  os << p << "_pos_valid," << p << "_pos_x," << p << "_pos_y," << p
     << "_vel_x," << p << "_vel_y," << p << "_pos_time," << p
     << "_angle_valid," << p << "_angle_deg," << p << "_angle_vel," << p
     << "_angle_time," << p << "_frame_id," << p << "_pos_alg," << p
     << "_angle_alg";
}

void OdometryLogger::_AppendOdometryData(std::ostream& os,
                                         const OdometryData& d) {
  bool pv = d.pos.has_value();
  bool av = d.angle.has_value();
  os << (pv ? 1 : 0) << ","
     << (pv ? d.pos->position.x : 0.f) << ","
     << (pv ? d.pos->position.y : 0.f) << ","
     << (pv ? d.pos->velocity.x : 0.f) << ","
     << (pv ? d.pos->velocity.y : 0.f) << ","
     << (pv ? d.pos->time : 0.0) << ","
     << (av ? 1 : 0) << ","
     << (av ? d.angle->angle.degrees() : 0.0) << ","
     << (av ? d.angle->velocity : 0.0) << ","
     << (av ? d.angle->time : 0.0) << ","
     << d.id << ","
     << (pv ? OdometryAlgToString(d.pos->algorithm) : "none") << ","
     << (av ? OdometryAlgToString(d.angle->algorithm) : "none");
}

void OdometryLogger::_WriteHeader() {
  if (!_file.is_open()) return;

  _file << "wall_clock,elapsed_s,";

  // All raw input algorithms
  const char* prefixes[] = {
      "us_blob",       "us_heuristic", "us_neural",      "us_neuralrot",
      "us_imu",        "us_human",     "us_lkflow",      "us_override",
      "us_opencv",     "them_blob",    "them_heuristic",  "them_human",
      "them_lkflow",   "them_override","them_opencv"};

  for (const char* p : prefixes) {
    _AppendOdometryHeader(_file, p);
    _file << ",";
  }

  // Fused outputs
  _AppendOdometryHeader(_file, "fused_robot");
  _file << ",";
  _AppendOdometryHeader(_file, "fused_opponent");
  _file << ",";

  // Back-annotation flags
  _file << "ba_swap_heuristic,ba_swap_blob,ba_force_robot_heuristic"
        << std::endl;

  _headerWritten = true;
}

void OdometryLogger::LogFusion(const RawInputs& inputs,
                               const FusionOutput& output,
                               double elapsedTime) {
  // Update the shared timestamp for TrackingWidget overlay
  {
    std::lock_guard<std::mutex> lock(_timestampMutex);
    std::ostringstream ts;
    int minutes = static_cast<int>(elapsedTime) / 60;
    double secs = elapsedTime - minutes * 60;
    ts << "T+" << minutes << ":" << std::fixed << std::setprecision(3)
       << std::setfill('0') << std::setw(6) << secs;
    _currentTimestamp = ts.str();
  }

  if (!_file.is_open()) return;
  if (!_headerWritten) _WriteHeader();

  // Wall clock
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()) %
                1000;
  auto now_tm = std::localtime(&now_time_t);

  _file << std::put_time(now_tm, "%H:%M:%S") << "." << std::setfill('0')
        << std::setw(3) << now_ms.count() << ",";

  _file << std::fixed << std::setprecision(4) << elapsedTime << ",";

  // Raw inputs in same order as header
  const OdometryData* sources[] = {
      &inputs.us_blob,       &inputs.us_heuristic, &inputs.us_neural,
      &inputs.us_neuralrot,  &inputs.us_imu,       &inputs.us_human,
      &inputs.us_lkflow,     &inputs.us_override,  &inputs.us_opencv,
      &inputs.them_blob,     &inputs.them_heuristic,&inputs.them_human,
      &inputs.them_lkflow,   &inputs.them_override, &inputs.them_opencv};

  for (const OdometryData* src : sources) {
    _AppendOdometryData(_file, *src);
    _file << ",";
  }

  // Fused outputs
  _AppendOdometryData(_file, output.robot);
  _file << ",";
  _AppendOdometryData(_file, output.opponent);
  _file << ",";

  // Back-annotation flags
  _file << (output.backAnnotate.swapHeuristic ? 1 : 0) << ","
        << (output.backAnnotate.swapBlob ? 1 : 0) << ","
        << (output.backAnnotate.forceRobotPos_Heuristic ? 1 : 0)
        << std::endl;
}

std::string OdometryLogger::GetTimestampString() {
  std::lock_guard<std::mutex> lock(_timestampMutex);
  return _currentTimestamp;
}
