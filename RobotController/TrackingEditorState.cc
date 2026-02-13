#include "TrackingEditorState.h"

TrackingEditorState& TrackingEditorState::GetInstance() {
  static TrackingEditorState instance;
  return instance;
}

TrackingEditorState::TrackingEditorState() {}

void TrackingEditorState::SetHeuristicEditing(bool enabled) {
  std::lock_guard<std::mutex> lock(_mutex);
  _heuristicEditing = enabled;
}

void TrackingEditorState::SetBlobEditing(bool enabled) {
  std::lock_guard<std::mutex> lock(_mutex);
  _blobEditing = enabled;
}

void TrackingEditorState::SetOpenCVEditing(bool enabled) {
  std::lock_guard<std::mutex> lock(_mutex);
  _openCVEditing = enabled;
}

void TrackingEditorState::ToggleHeuristicEditing() {
  std::lock_guard<std::mutex> lock(_mutex);
  _heuristicEditing = !_heuristicEditing;
}

void TrackingEditorState::ToggleBlobEditing() {
  std::lock_guard<std::mutex> lock(_mutex);
  _blobEditing = !_blobEditing;
}

void TrackingEditorState::ToggleOpenCVEditing() {
  std::lock_guard<std::mutex> lock(_mutex);
  _openCVEditing = !_openCVEditing;
}

void TrackingEditorState::ToggleLKFlowEditing() {
  std::lock_guard<std::mutex> lock(_mutex);
  _lkFlowEditing = !_lkFlowEditing;
}

bool TrackingEditorState::IsHeuristicEditing() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _heuristicEditing;
}

bool TrackingEditorState::IsBlobEditing() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _blobEditing;
}

bool TrackingEditorState::IsOpenCVEditing() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _openCVEditing;
}

bool TrackingEditorState::IsLKFlowEditing() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _lkFlowEditing;
}
