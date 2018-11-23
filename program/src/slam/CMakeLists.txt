
add_library(
    slam
    STATIC
    FinitePriorityQueue.h
    Debug.cpp
    Debug.h
    Misc.cpp
    Misc.h
    SLAMModule.cpp
    SLAMModule.h
    SLAMModuleFeatures.cpp
    SLAMModuleFeatures.h
    SLAMModuleTemporalMatcher.cpp
    SLAMModuleTemporalMatcher.h
    SLAMModuleStereoMatcher.cpp
    SLAMModuleStereoMatcher.h
    SLAMModuleTriangulation.cpp
    SLAMModuleTriangulation.h
    SLAMModuleAlignment.cpp
    SLAMModuleAlignment.h
    SLAMPyramid.h
    SLAMProject.cpp
    SLAMProject.h
    SLAMConfig.cpp
    SLAMConfig.h
    SLAMDataStructures.h
    SLAMSystem.cpp
    SLAMSystem.h
)

target_link_libraries(
    slam
    PUBLIC
    Threads::Threads
    g2o::core
    Qt5::Core
    Qt5::Widgets
    Eigen3::Eigen
    opencv_core
    opencv_imgproc
    opencv_features2d
    opencv_calib3d
    opencv_video
    video
    calibration
    buildinfo)

target_include_directories(slam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

