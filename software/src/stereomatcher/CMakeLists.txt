
add_library(
    stereomatcher
    STATIC
    StereoMatcher.h
    StereoMatcher.cpp
    Primitives.cu
)

target_link_libraries(
    stereomatcher
    PUBLIC
    opencv_core
    opencv_calib3d)

target_include_directories(stereomatcher INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

