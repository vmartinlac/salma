
add_library(
    slam
    STATIC
    FinitePriorityQueue.h
    SLAMDataStructures.h
    SLAMConfiguration.cpp
    SLAMConfiguration.h
    SLAMEngine.cpp
    SLAMEngine.h
    SLAMContext.cpp
    SLAMContext.h
    SLAMModule.cpp
    SLAMModule.h
    SLAMModuleOpticalFlow.cpp
    SLAMModuleOpticalFlow.h
    SLAMModuleAlignment.cpp
    SLAMModuleAlignment.h
    SLAMModuleFeatures.cpp
    SLAMModuleFeatures.h
    SLAMModuleStereoMatcher.cpp
    SLAMModuleStereoMatcher.h
    SLAMModuleTriangulation.cpp
    SLAMModuleTriangulation.h
    SLAMModuleDenseReconstruction.cpp
    SLAMModuleDenseReconstruction.h
)

target_link_libraries(
    slam
    PUBLIC
    Threads::Threads
    Qt5::Core
    Qt5::Widgets
    Eigen3::Eigen
    opencv_imgproc
    opencv_features2d
    opencv_calib3d
    opencv_video
    video
    mvpnp
    calibration
    buildinfo
)

target_include_directories(slam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

