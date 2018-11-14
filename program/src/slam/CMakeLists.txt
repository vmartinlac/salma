
add_library(
    slam
    STATIC
    Misc.cpp
    Misc.h
    SLAMConfig.cpp
    SLAMConfig.h
    SLAMDataStructures.h
    SLAMSystem.cpp
    SLAMSystem.h
    StereoMatcher.cpp
    StereoMatcher.h
)

target_link_libraries(
    slam
    PUBLIC
    Threads::Threads
    g2o::core
    Qt5::Core
    Eigen3::Eigen
    opencv_core
    opencv_imgproc
    opencv_features2d
    opencv_calib3d
    video
    calibration
    buildinfo
    dbg # temporary, for development purpose!
    features)

target_include_directories(slam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

