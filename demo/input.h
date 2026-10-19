#pragma once

namespace muli3
{

struct InputButtonState
{
    bool down = false;
    bool pressed = false;
    bool released = false;
};

class Input
{
public:
    static void Reset(double x, double y)
    {
        pendingKeys.fill({});
        pendingButtons.fill({});

        keys = pendingKeys;
        buttons = pendingButtons;

        mouseScroll.SetZero();
        pendingScroll.SetZero();
        ResetMousePosition(x, y);
    }

    // Publish collected events before game/UI input is consumed, once per frame.
    static void BeginFrame()
    {
        keys = pendingKeys;
        buttons = pendingButtons;

        for (InputButtonState& key : pendingKeys)
            key.pressed = key.released = false;
        for (InputButtonState& button : pendingButtons)
            button.pressed = button.released = false;

        mousePosition = { float(cursorX), float(cursorY) };
        mouseDelta = { float(deltaX), float(deltaY) };
        mouseScroll = pendingScroll;

        deltaX = deltaY = 0;
        pendingScroll.SetZero();
    }

    static void OnKey(int key, int action)
    {
        if (key < 0 || key > GLFW_KEY_LAST || action == GLFW_REPEAT) return;
        UpdateButton(pendingKeys[key], action);
    }

    static void OnMouseButton(int button, int action)
    {
        if (button < 0 || button > GLFW_MOUSE_BUTTON_LAST) return;
        UpdateButton(pendingButtons[button], action);
    }

    static void OnCursorPosition(double x, double y)
    {
        deltaX += x - cursorX;
        deltaY += y - cursorY;
        cursorX = x;
        cursorY = y;
    }

    static void OnScroll(double x, double y)
    {
        pendingScroll.x += float(x);
        pendingScroll.y += float(y);
    }

    // Cursor-mode changes establish a new baseline, without synthetic look motion.
    static void ResetMousePosition(double x, double y)
    {
        cursorX = x;
        cursorY = y;
        deltaX = deltaY = 0;
        mousePosition = { float(x), float(y) };
        mouseDelta.SetZero();
    }

    static bool IsKeyDown(int key)
    {
        return key >= 0 && key <= GLFW_KEY_LAST ? keys[key].down : false;
    }

    static bool IsKeyPressed(int key)
    {
        return key >= 0 && key <= GLFW_KEY_LAST ? keys[key].pressed : false;
    }

    static bool IsMouseDown(int button)
    {
        return button >= 0 && button <= GLFW_MOUSE_BUTTON_LAST ? buttons[button].down : false;
    }

    static bool IsMousePressed(int button)
    {
        return button >= 0 && button <= GLFW_MOUSE_BUTTON_LAST ? buttons[button].pressed : false;
    }

    static bool IsMouseReleased(int button)
    {
        return button >= 0 && button <= GLFW_MOUSE_BUTTON_LAST ? buttons[button].released : false;
    }

    static Vec2 GetMousePosition()
    {
        return mousePosition;
    }

    static Vec2 GetMouseDelta()
    {
        return mouseDelta;
    }

    static Vec2 GetMouseScroll()
    {
        return mouseScroll;
    }

private:
    static void UpdateButton(InputButtonState& button, int action)
    {
        if (action == GLFW_PRESS)
        {
            button.pressed |= !button.down;
        }
        else
        {
            button.released |= button.down;
        }

        button.down = action == GLFW_PRESS;
    }

    inline static std::array<InputButtonState, GLFW_KEY_LAST + 1> pendingKeys{}, keys{};
    inline static std::array<InputButtonState, GLFW_MOUSE_BUTTON_LAST + 1> pendingButtons{}, buttons{};

    inline static double cursorX = 0, cursorY = 0;
    inline static double deltaX = 0, deltaY = 0;

    inline static Vec2 mousePosition{ 0.0f, 0.0f };
    inline static Vec2 mouseDelta{ 0.0f, 0.0f };
    inline static Vec2 mouseScroll{ 0.0f, 0.0f };
    inline static Vec2 pendingScroll{ 0.0f, 0.0f };
};

} // namespace muli3
