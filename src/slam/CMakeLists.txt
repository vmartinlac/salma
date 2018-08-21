
add_library(
    slam
    STATIC
    SLAMEngine.h
    SLAMEngine.cpp
    SLAMEngineImpl.h
    SLAMEngineImpl.cpp
    SLAMParameters.h
    SLAMParameters.cpp
    SLAMOutput.h
    SLAMOutput.cpp
)

target_link_libraries(
    slam
    PUBLIC
    camera
    target
    Qt5::Core
    Vimba
    opencv_core
    opencv_imgproc
    opencv_features2d
    opencv_calib3d
    Eigen3::Eigen)

target_include_directories(slam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

