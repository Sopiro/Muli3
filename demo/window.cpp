#include "window.h"
#include "input.h"

namespace muli3
{

Window::Window(int32 width, int32 height, const char* title)
    : windowSize{ width, height }
{
    glfwSetErrorCallback(ErrorCallback);
    if (!glfwInit())
    {
        std::fprintf(stderr, "Failed to initialize glfw\n");
        std::exit(1);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    handle = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!handle)
    {
        std::fprintf(stderr, "Failed to create glfw window\n");
        glfwTerminate();
        std::exit(1);
    }

    const GLFWvidmode* videoMode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    if (videoMode)
    {
        glfwSetWindowMonitor(
            handle, nullptr, (videoMode->width / 2) - (width / 2), (videoMode->height / 2) - (height / 2), width, height,
            GLFW_DONT_CARE
        );
        refreshRate = videoMode->refreshRate;
    }

    glfwMakeContextCurrent(handle);
    glfwSwapInterval(0);

    if (!gladLoadGLLoader(GLADloadproc(glfwGetProcAddress)))
    {
        std::fprintf(stderr, "Failed to initialize glad\n");
        glfwTerminate();
        std::exit(1);
    }

    glfwSetWindowUserPointer(handle, this);
    glfwSetWindowSizeCallback(handle, OnWindowSize);
    glfwSetFramebufferSizeCallback(handle, OnFramebufferSize);
    glfwSetKeyCallback(handle, OnKey);
    glfwSetCharCallback(handle, OnChar);
    glfwSetMouseButtonCallback(handle, OnMouseButton);
    glfwSetCursorPosCallback(handle, OnCursorPosition);
    glfwSetScrollCallback(handle, OnScroll);
    glfwGetWindowSize(handle, &windowSize.x, &windowSize.y);
    glfwGetFramebufferSize(handle, &framebufferSize.x, &framebufferSize.y);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(handle, false);
    ImGui_ImplOpenGL3_Init("#version 450");

    double x = 0.0, y = 0.0;
    glfwGetCursorPos(handle, &x, &y);
    Input::Reset(x, y);
}

Window::~Window()
{
    if (handle)
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        glfwDestroyWindow(handle);
        glfwTerminate();
    }
}

void Window::SetFramebufferSizeChangeCallback(std::function<void(int32, int32)> callback)
{
    framebufferSizeChangeCallback = std::move(callback);
}

void Window::BeginFrame() const
{
    glfwPollEvents();
    Input::BeginFrame();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    glViewport(0, 0, framebufferSize.x, framebufferSize.y);
}

void Window::EndFrame() const
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(handle);
}

void Window::SetCursorHidden(bool hidden)
{
    if (GetCursorHidden() == hidden)
    {
        return;
    }

    if (hidden)
    {
        glfwSetInputMode(handle, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    }
    else
    {
        glfwSetInputMode(handle, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        // glfwSetCursorPos(handle, windowSize.x * 0.5f, windowSize.y * 0.5f);
    }

    if (glfwRawMouseMotionSupported())
    {
        glfwSetInputMode(handle, GLFW_RAW_MOUSE_MOTION, hidden ? GLFW_TRUE : GLFW_FALSE);
    }

    double x = 0.0;
    double y = 0.0;
    glfwGetCursorPos(handle, &x, &y);
    Input::ResetMousePosition(x, y);
}

void Window::ErrorCallback(int error, const char* description)
{
    std::fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

void Window::OnWindowSize(GLFWwindow* window, int width, int height)
{
    Window* self = (Window*)glfwGetWindowUserPointer(window);
    self->windowSize.Set(width, height);
}

void Window::OnFramebufferSize(GLFWwindow* window, int width, int height)
{
    Window* self = (Window*)glfwGetWindowUserPointer(window);
    self->framebufferSize.Set(width, height);

    if (self->framebufferSizeChangeCallback)
    {
        self->framebufferSizeChangeCallback(width, height);
    }
}

void Window::OnKey(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
    Input::OnKey(key, action);
}

void Window::OnChar(GLFWwindow* window, unsigned int c)
{
    ImGui_ImplGlfw_CharCallback(window, c);
}

void Window::OnMouseButton(GLFWwindow* window, int button, int action, int mods)
{
    ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
    Input::OnMouseButton(button, action);
}

void Window::OnCursorPosition(GLFWwindow* window, double x, double y)
{
    ImGui_ImplGlfw_CursorPosCallback(window, x, y);
    Input::OnCursorPosition(x, y);
}

void Window::OnScroll(GLFWwindow* window, double x, double y)
{
    ImGui_ImplGlfw_ScrollCallback(window, x, y);
    Input::OnScroll(x, y);
}

} // namespace muli3
