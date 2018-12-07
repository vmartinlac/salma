
add_library(
    calibration
    STATIC
    Serialization.h
    Serialization.cpp
    CameraCalibrationData.h
    CameraCalibrationData.cpp
    StereoRigCalibrationData.h
    StereoRigCalibrationData.cpp
    TwoViewGeometry.h
    TwoViewGeometry.cpp
)

target_include_directories(calibration INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(calibration PUBLIC Qt5::Core Eigen3::Eigen Sophus::Sophus opencv_core)

