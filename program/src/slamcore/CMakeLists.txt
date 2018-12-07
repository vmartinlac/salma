
add_library(
    slamcore
    STATIC
    SLAMProject.cpp
    SLAMProject.h
    SLAMDataStructures.h
)

target_link_libraries(
    slamcore
    PUBLIC
    Qt5::Core
    Qt5::Widgets
    Eigen3::Eigen
    opencv_core
    video
    calibration
)

target_include_directories(slamcore INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

