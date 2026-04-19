#pragma once

#include "common.h"

namespace muli3
{

class Input
{
public:
    static void Update()
    {
        previousKeys = currentKeys;
        previousButtons = currentButtons;
        mouseDelta = currentMousePosition - previousMousePosition;
        previousMousePosition = currentMousePosition;
        mouseScroll.SetZero();
    }

    static bool IsKeyDown(int key)
    {
        return key >= 0 && key <= GLFW_KEY_LAST ? currentKeys[key] : false;
    }

    static bool IsKeyPressed(int key)
    {
        return key >= 0 && key <= GLFW_KEY_LAST ? currentKeys[key] && !previousKeys[key] : false;
    }

    static bool IsMouseDown(int button)
    {
        return button >= 0 && button <= GLFW_MOUSE_BUTTON_LAST ? currentButtons[button] : false;
    }

    static bool IsMousePressed(int button)
    {
        return button >= 0 && button <= GLFW_MOUSE_BUTTON_LAST ? currentButtons[button] && !previousButtons[button] : false;
    }

    static bool IsMouseReleased(int button)
    {
        return button >= 0 && button <= GLFW_MOUSE_BUTTON_LAST ? !currentButtons[button] && previousButtons[button] : false;
    }

    static Vec2 GetMousePosition()
    {
        return currentMousePosition;
    }

    static Vec2 GetMouseDelta()
    {
        return mouseDelta;
    }

    static Vec2 GetMouseScroll()
    {
        return mouseScroll;
    }

    inline static std::array<bool, GLFW_KEY_LAST + 1> currentKeys = {};
    inline static std::array<bool, GLFW_KEY_LAST + 1> previousKeys = {};
    inline static std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> currentButtons = {};
    inline static std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> previousButtons = {};
    inline static Vec2 currentMousePosition{ 0.0f, 0.0f };
    inline static Vec2 previousMousePosition{ 0.0f, 0.0f };
    inline static Vec2 mouseDelta{ 0.0f, 0.0f };
    inline static Vec2 mouseScroll{ 0.0f, 0.0f };
};

} // namespace muli3
