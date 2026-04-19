#pragma once

#include "common.h"
#include "input.h"

namespace muli3
{

class Window : NonCopyable
{
public:
    ~Window();

    void SetFramebufferSizeChangeCallback(std::function<void(int32, int32)> callback);
    Vec2 GetWindowSize() const;
    int32 GetRefreshRate() const;

    bool GetCursorHidden() const;
    void SetCursorHidden(bool hidden);

    bool ShouldClose() const;
    void BeginFrame(const Vec3& clearColor) const;
    void EndFrame() const;

    GLFWwindow* GetNativeHandle() const;

    static Window* Get();
    static Window* Init(int32 width, int32 height, const char* title);
    static void Shutdown();

private:
    Window(int32 width, int32 height, const char* title);

    inline static std::unique_ptr<Window> window;

    GLFWwindow* handle = nullptr;
    int32 width = 0;
    int32 height = 0;
    int32 refreshRate = 60;
    std::function<void(int32, int32)> framebufferSizeChangeCallback = nullptr;

    static void ErrorCallback(int error, const char* description);
    static void OnFramebufferSize(GLFWwindow* window, int width, int height);
    static void OnKey(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void OnMouseButton(GLFWwindow* window, int button, int action, int mods);
    static void OnCursorPosition(GLFWwindow* window, double x, double y);
    static void OnScroll(GLFWwindow* window, double x, double y);
};

inline Window* Window::Get()
{
    return window.get();
}

inline Window* Window::Init(int32 width, int32 height, const char* title)
{
    if (!window)
    {
        window = std::unique_ptr<Window>(new Window(width, height, title));
    }

    return window.get();
}

inline void Window::Shutdown()
{
    window.reset();
}

inline bool Window::ShouldClose() const
{
    return glfwWindowShouldClose(handle) != 0;
}

inline Vec2 Window::GetWindowSize() const
{
    return Vec2{ (float)width, (float)height };
}

inline int32 Window::GetRefreshRate() const
{
    return refreshRate;
}

inline bool Window::GetCursorHidden() const
{
    return glfwGetInputMode(handle, GLFW_CURSOR) == GLFW_CURSOR_DISABLED;
}

inline GLFWwindow* Window::GetNativeHandle() const
{
    return handle;
}

} // namespace muli3
