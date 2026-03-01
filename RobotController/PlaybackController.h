#pragma once
#include <string>
#include <mutex>
#include <cstdint>

enum class StepRequest { None, StepForward, StepBackward };

/**
 * @class PlaybackController
 * @brief Manages video playback state in a thread-safe, testable manner.
 * Uses frame units only (no time-based control).
 */
class PlaybackController
{
public:
    static PlaybackController& GetInstance();

    // Playback control
    void Play();
    void Stop();
    void TogglePause();
    void Restart();
    void ToggleReverse();

    // Frame-by-frame stepping (when paused)
    void RequestStepForward();
    void RequestStepBackward();
    StepRequest ConsumeStepRequest();

    void SetSpeed(float speed);
    void SetFile(const std::string& filename);

    // Frame-based API
    int64_t GetFrame() const;
    void SetFrame(int64_t frame);
    void SetFrameCount(int64_t count);
    void RequestSeekFrame(int64_t frame);
    bool ConsumeSeekFrame(int64_t& outFrame);
    int64_t GetFrameCount() const;
    
    // Getters (thread-safe)
    bool IsPlaying() const;
    bool IsPaused() const;
    bool IsReversing() const;
    bool ShouldRestart();  // Returns and clears restart flag
    float GetSpeed() const;
    std::string GetFile() const;
    bool HasFileChanged();  // Returns and clears file changed flag

private:
    PlaybackController();
    ~PlaybackController() = default;
    
    // Prevent copying
    PlaybackController(const PlaybackController&) = delete;
    PlaybackController& operator=(const PlaybackController&) = delete;

    mutable std::mutex _mutex;
    
    // State variables
    bool _playing = true;
    bool _paused = false;
    bool _reversing = false;
    bool _restart = false;
    float _speed = 1.0f;
    std::string _filename = "./Recordings/GP4_BB_Live_ProvingGround_03092024_1.mp4";
    bool _fileChanged = true;
    int64_t _frame = 0;
    int64_t _frameCount = 1;
    bool _seekRequested = false;
    int64_t _seekFrame = 0;
    StepRequest _stepRequest = StepRequest::None;
};
