
add_library(
    salma_slam
    SHARED
    FinitePriorityQueue.h
    SLAMDataStructures.cpp
    SLAMDataStructures.h
    SLAMConfiguration.cpp
    SLAMConfiguration.h
    SLAMEngine.cpp
    SLAMEngine.h
    SLAMContext.cpp
    SLAMContext.h
    SLAMModule.cpp
    SLAMModule.h
    SLAMModuleFeatures.cpp
    SLAMModuleFeatures.h
    SLAMModuleTemporalMatcher.cpp
    SLAMModuleTemporalMatcher.h
    SLAMModuleAlignment.cpp
    SLAMModuleAlignment.h
    SLAMModuleLBA.cpp
    SLAMModuleLBA.h
    SLAMModuleEKF.cpp
    SLAMModuleEKF.h
    SLAMModuleStereoMatcher.cpp
    SLAMModuleStereoMatcher.h
    SLAMModuleTriangulation.cpp
    SLAMModuleTriangulation.h
    #SLAMModuleDenseReconstruction.cpp
    #SLAMModuleDenseReconstruction.h
    SLAMDebug.cpp
    SLAMDebug.h
    SLAMMath.cpp
    SLAMMath.h
)

target_link_libraries(
    salma_slam
    PUBLIC
    #Threads::Threads
    Qt5::Core
    Qt5::Widgets
    Eigen3::Eigen
    opencv_imgproc
    opencv_features2d
    opencv_calib3d
    opencv_video
    #opencv_cudastereo
    #opencv_cudaimgproc
    salma_video
    salma_mvpnp
    salma_calibration
    salma_buildinfo
    #salma_stereomatcher
)

target_include_directories(salma_slam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS salma_slam LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

