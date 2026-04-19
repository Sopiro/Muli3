set(HEADER_FILES
    ../include/muli3/types.h
    ../include/muli3/common.h
    ../include/muli3/format.h
    ../include/muli3/math.h
    ../include/muli3/aabb.h
    ../include/muli3/settings.h
    ../include/muli3/shape.h
    ../include/muli3/sphere.h
    ../include/muli3/rigidbody.h
    ../include/muli3/world.h
    ../include/muli3/muli3.h
)

set(SOURCE_FILES
    util/math.cpp
    geometry/sphere.cpp
    dynamics/rigidbody.cpp
    dynamics/world.cpp
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${HEADER_FILES})

add_library(muli3 ${SOURCE_FILES} ${HEADER_FILES})

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

if(MSVC)
    target_compile_options(muli3 PRIVATE /W4 /wd4819)
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
