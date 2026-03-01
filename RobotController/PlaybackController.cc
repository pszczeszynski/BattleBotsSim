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

void PlaybackController::RequestStepForward()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _stepRequest = StepRequest::StepForward;
}

void PlaybackController::RequestStepBackward()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _stepRequest = StepRequest::StepBackward;
}

StepRequest PlaybackController::ConsumeStepRequest()
{
    std::lock_guard<std::mutex> lock(_mutex);
    StepRequest req = _stepRequest;
    _stepRequest = StepRequest::None;
    return req;
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
        _frame = 0;
    }
}

int64_t PlaybackController::GetFrame() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _frame;
}

void PlaybackController::SetFrame(int64_t frame)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _frame = frame;
}

void PlaybackController::SetFrameCount(int64_t count)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _frameCount = (count > 0) ? count : 1;
}

void PlaybackController::RequestSeekFrame(int64_t frame)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _seekRequested = true;
    _seekFrame = frame;
}

bool PlaybackController::ConsumeSeekFrame(int64_t& outFrame)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_seekRequested)
        return false;
    _seekRequested = false;
    outFrame = _seekFrame;
    return true;
}

int64_t PlaybackController::GetFrameCount() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _frameCount;
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

