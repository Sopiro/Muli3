file(GLOB_RECURSE HEADER_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${HEADER_FILES})

add_library(muli3 ${HEADER_FILES} ${SOURCE_FILES})

target_include_directories(muli3
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(muli3 PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(MULI3_PROFILE)
    include(FetchContent)

    target_compile_definitions(muli3 PUBLIC MULI3_PROFILE)

    set(TRACY_DELAYED_INIT ON CACHE BOOL "Enable delayed init for Tracy" FORCE)
    set(TRACY_MANUAL_LIFETIME ON CACHE BOOL "Enable manual lifetime for Tracy" FORCE)
    set(TRACY_NO_SYSTEM_TRACING ON CACHE BOOL "Disable Tracy system tracing" FORCE)

    FetchContent_Declare(
        tracy
        GIT_REPOSITORY https://github.com/wolfpld/tracy.git
        GIT_TAG v0.13.1
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
    )
    FetchContent_MakeAvailable(tracy)

    target_link_libraries(muli3 PUBLIC TracyClient)
endif()

if(MSVC)
    target_compile_options(muli3 PRIVATE /W4 /WX /wd4819)
else()
    target_compile_options(muli3 PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(
    TARGETS muli3
    EXPORT muli3Config
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    EXPORT muli3Config
    NAMESPACE muli3::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/muli3
)
