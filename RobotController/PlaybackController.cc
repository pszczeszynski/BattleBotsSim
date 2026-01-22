#include "PlaybackController.h"

PlaybackController& PlaybackController::GetInstance()
{
    static PlaybackController instance;
    return instance;
}

PlaybackController::PlaybackController()
{
}

void PlaybackController::Play()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _playing = true;
    _paused = false;
}

void PlaybackController::Stop()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _playing = false;
}

void PlaybackController::TogglePause()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _paused = !_paused;
}

void PlaybackController::Restart()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _restart = true;
}

void PlaybackController::ToggleReverse()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _reversing = !_reversing;
}

void PlaybackController::SetSpeed(float speed)
{
    std::lock_guard<std::mutex> lock(_mutex);
    // Ensure speed is positive
    _speed = (speed > 0.0f) ? speed : 1.0f;
}

void PlaybackController::SetFile(const std::string& filename)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_filename != filename)
    {
        _filename = filename;
        _fileChanged = true;
        _videoPositionSeconds = 0.0f;
    }
}

void PlaybackController::SetVideoLength(float lengthSeconds)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _videoLengthSeconds = lengthSeconds;
}

void PlaybackController::SetVideoPosition(float positionSeconds)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _videoPositionSeconds = positionSeconds;
}

bool PlaybackController::IsPlaying() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _playing;
}

bool PlaybackController::IsPaused() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _paused;
}

bool PlaybackController::IsReversing() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _reversing;
}

bool PlaybackController::ShouldRestart()
{
    std::lock_guard<std::mutex> lock(_mutex);
    bool shouldRestart = _restart;
    _restart = false;  // Clear the flag
    return shouldRestart;
}

float PlaybackController::GetSpeed() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _speed;
}

std::string PlaybackController::GetFile() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _filename;
}

bool PlaybackController::HasFileChanged()
{
    std::lock_guard<std::mutex> lock(_mutex);
    bool changed = _fileChanged;
    _fileChanged = false;  // Clear the flag
    return changed;
}

float PlaybackController::GetVideoLength() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _videoLengthSeconds;
}

float PlaybackController::GetVideoPosition() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _videoPositionSeconds;
}
