include(FetchContent)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
)

set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

FetchContent_Populate(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.92.4
)

file(GLOB IMGUI_HEADER_FILES ${imgui_SOURCE_DIR}/*.h)
file(GLOB IMGUI_SOURCE_FILES ${imgui_SOURCE_DIR}/*.cpp)

add_library(imgui
    ${IMGUI_HEADER_FILES}
    ${IMGUI_SOURCE_FILES}
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(imgui PUBLIC glfw)

set_target_properties(imgui PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

file(GLOB_RECURSE DEMO_HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE DEMO_SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

set(DEMO_GLAD_FILES
    glad/include/glad/glad.h
    glad/include/KHR/khrplatform.h
    glad/src/glad.c
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "demo" FILES ${DEMO_HEADER_FILES} ${DEMO_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/glad" PREFIX "glad" FILES ${DEMO_GLAD_FILES})

add_executable(demo ${DEMO_HEADER_FILES} ${DEMO_GLAD_FILES} ${DEMO_SOURCE_FILES})

target_precompile_headers(demo PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/common.h>"
)

target_include_directories(demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/glad/include
)

target_link_libraries(demo PUBLIC
    muli3
    glfw
    imgui
)

set_target_properties(demo PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(MSVC)
    target_compile_options(glfw PRIVATE /wd4819)
    target_compile_options(imgui PRIVATE /wd4819)
    target_compile_options(demo PRIVATE /wd4819)
else()
    target_compile_options(imgui PRIVATE -Wno-missing-field-initializers)
endif()
