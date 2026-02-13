#pragma once
#include <mutex>

/**
 * @class TrackingEditorState
 * @brief Manages which tracking algorithms are editable in the UI.
 */
class TrackingEditorState {
 public:
  static TrackingEditorState& GetInstance();

  // Enable/disable editing for specific algorithms
  void SetHeuristicEditing(bool enabled);
  void SetBlobEditing(bool enabled);
  void SetOpenCVEditing(bool enabled);
  void SetLKFlowEditing(bool enabled);

  void ToggleHeuristicEditing();
  void ToggleBlobEditing();
  void ToggleOpenCVEditing();
  void ToggleLKFlowEditing();

  // Query current state (thread-safe)
  bool IsHeuristicEditing() const;
  bool IsBlobEditing() const;
  bool IsOpenCVEditing() const;
  bool IsLKFlowEditing() const;

 private:
  TrackingEditorState();
  ~TrackingEditorState() = default;

  // Prevent copying
  TrackingEditorState(const TrackingEditorState&) = delete;
  TrackingEditorState& operator=(const TrackingEditorState&) = delete;

  mutable std::mutex _mutex;

  // State variables
  bool _heuristicEditing = true;
  bool _blobEditing = true;
  bool _openCVEditing = true;
  bool _lkFlowEditing = true;
};
