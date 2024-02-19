#include "InputState.h"
#include <iostream>
InputState InputState::instance;

#define KEY_START_OFFSET 512

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

void InputState::UpdateMouseButtonState(int button, bool isDown)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (button >= 0 && button < mouseButtonStates_.size())
    {
        mouseButtonStates_[button] = isDown;
    }
}

// Check if a key is down; can be called from any thread
bool InputState::IsKeyDown(ImGuiKey key) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    int key_int = (int) key - KEY_START_OFFSET;
    if (key_int >= 0 && key_int < keyStates_.size())
    {
        return keyStates_[(int) key_int];
    }
    return false;
}

bool InputState::IsMouseDown(int button) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (button >= 0 && button < mouseButtonStates_.size())
    {
        return mouseButtonStates_[button];
    }
    return false;
}

// Update all key states based on ImGui; should be called from the UI thread
void InputState::UpdateAllKeyStates()
{
    // we start at 512 because the first 512 are not keys (see the ImGuiKey enum
    // in imgui.h)
    for (int i = 0; i < keyStates_.size(); ++i)
    {
        // Assuming ImGui::IsKeyDown is available and ImGui is properly initialized
        UpdateKeyState(i, ImGui::IsKeyDown(static_cast<ImGuiKey>(i + KEY_START_OFFSET)));
    }

    // update mouse button states
    for (int i = 0; i < 3; i++)
    {
        UpdateMouseButtonState(i, ImGui::IsMouseDown(i));
    }
}
