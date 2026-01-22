#pragma once
#include <string>
#include <mutex>

/**
 * @class PlaybackController
 * @brief Manages video playback state in a thread-safe, testable manner.
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
    
    void SetSpeed(float speed);
    void SetFile(const std::string& filename);
    void SetVideoLength(float lengthSeconds);
    void SetVideoPosition(float positionSeconds);
    
    // Getters (thread-safe)
    bool IsPlaying() const;
    bool IsPaused() const;
    bool IsReversing() const;
    bool ShouldRestart();  // Returns and clears restart flag
    float GetSpeed() const;
    std::string GetFile() const;
    bool HasFileChanged();  // Returns and clears file changed flag
    float GetVideoLength() const;
    float GetVideoPosition() const;

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
    float _videoLengthSeconds = 0.0f;
    float _videoPositionSeconds = 0.0f;
};
