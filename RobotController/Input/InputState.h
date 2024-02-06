#pragma once

#include <mutex>
#include <array>
#include <imgui.h>

class InputState
{
public:
    InputState() = default;

    static InputState& GetInstance();
    // Update the key state; should be called from the UI thread
    void UpdateKeyState(int key, bool isDown);
    // Check if a key is down; can be called from any thread
    bool IsKeyDown(ImGuiKey key) const;

    // Update all key states based on ImGui; should be called from the UI thread
    void UpdateAllKeyStates();

private:
    mutable std::mutex mutex_; // mutable because IsKeyDown is const
    std::array<bool, 1024> keyStates_{}; // Adjust the size based on your needs

    static InputState instance;
};
