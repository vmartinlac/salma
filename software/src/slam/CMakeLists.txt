
add_library(
    slam
    STATIC
    FinitePriorityQueue.h
    Debug.cpp
    Debug.h
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
    SLAMProject.cpp
    SLAMProject.h
    SLAMDataStructures.h
    #SLAMSystem.cpp
    #SLAMSystem.h
    #SLAMReconstructionDB.h
    #SLAMReconstructionDB.cpp
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

