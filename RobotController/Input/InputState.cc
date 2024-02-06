#include "InputState.h"

InputState InputState::instance;

InputState &InputState::GetInstance()
{
    return instance;
}


// Update the key state; should be called from the UI thread
void InputState::UpdateKeyState(int key, bool isDown)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (key >= 0 && key < keyStates_.size())
    {
        keyStates_[key] = isDown;
    }
}

// Check if a key is down; can be called from any thread
bool InputState::IsKeyDown(ImGuiKey key) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (key >= 0 && key < keyStates_.size())
    {
        return keyStates_[(int) key];
    }
    return false;
}

// Update all key states based on ImGui; should be called from the UI thread
void InputState::UpdateAllKeyStates()
{
    // we start at 512 because the first 512 are not keys (see the ImGuiKey enum
    // in imgui.h)
    for (int i = 512; i < keyStates_.size(); ++i)
    {
        // Assuming ImGui::IsKeyDown is available and ImGui is properly initialized
        UpdateKeyState(i, ImGui::IsKeyDown(static_cast<ImGuiKey>(i)));
    }
}
