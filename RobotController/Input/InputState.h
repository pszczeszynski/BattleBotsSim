#pragma once

#include <mutex>
#include <array>
#include <imgui.h>

class InputState
{
public:
    InputState() = default;

    static InputState& GetInstance();
    // Check if a key is down; can be called from any thread
    bool IsKeyDown(ImGuiKey key) const;

    bool IsMouseDown(int button) const;

    // Update all key states based on ImGui; should be called from the UI thread
    void UpdateAllKeyStates();

private:
    // Update the key state; should be called from the UI thread
    void UpdateKeyState(int key, bool isDown);
    void UpdateMouseButtonState(int button, bool isDown);

    mutable std::mutex mutex_; // mutable because IsKeyDown is const
    std::array<bool, 100> keyStates_{}; // Adjust the size based on your needs

    static InputState instance;

    std::array<bool, 3> mouseButtonStates_{};
};
