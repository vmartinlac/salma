
add_library(
    target
    STATIC
    Tracker.h
    Tracker.cpp
)

target_link_libraries(target PUBLIC opencv_core opencv_imgproc opencv_features2d nanoflann::nanoflann)
target_include_directories(target INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

