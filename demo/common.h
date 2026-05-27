#pragma once

#include <muli3/muli3.h>

#ifndef GLFW_INCLUDE_NONE
    #define GLFW_INCLUDE_NONE
#endif

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include <imgui.h>

#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

namespace muli3
{

class NonCopyable
{
public:
    NonCopyable() = default;
    NonCopyable(const NonCopyable&) = delete;
    NonCopyable& operator=(const NonCopyable&) = delete;
};

} // namespace muli3
