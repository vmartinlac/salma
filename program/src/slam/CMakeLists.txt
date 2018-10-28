
add_library(
    slam
    STATIC
    slam.cpp
)

target_link_libraries(
    slam
    PUBLIC
    video
    Qt5::Core
    opencv_core
    opencv_imgproc
    opencv_features2d
    opencv_calib3d
    calibration
    Eigen3::Eigen)

target_include_directories(slam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

