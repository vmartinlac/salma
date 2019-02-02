
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
    SLAMModule1Features.cpp
    SLAMModule1Features.h
    SLAMModule1TemporalMatcher.cpp
    SLAMModule1TemporalMatcher.h
    SLAMModule1Alignment.cpp
    SLAMModule1Alignment.h
    SLAMModule1LBA.cpp
    SLAMModule1LBA.h
    SLAMModule1StereoMatcher.cpp
    SLAMModule1StereoMatcher.h
    SLAMModule1Triangulation.cpp
    SLAMModule1Triangulation.h
    SLAMModule1DenseReconstruction.cpp
    SLAMModule1DenseReconstruction.h
    #SLAMModule2OpticalFlow.cpp
    #SLAMModule2OpticalFlow.h
    SLAMModule2EKF.cpp
    SLAMModule2EKF.h
    SLAMDebug.cpp
    SLAMDebug.h
    SLAMMath.cpp
    SLAMMath.h
    EdgeProjectP2R.cpp
    EdgeProjectP2R.h
)

target_link_libraries(
    salma_slam
    PUBLIC
    g2o::core
    g2o::types_sba
    g2o::solver_eigen
    Qt5::Core
    #Qt5::Widgets
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
    opencv_imgproc
    opencv_features2d
    opencv_calib3d
    opencv_video
    #Threads::Threads
    salma_video
    salma_mvpnp
    salma_calibration
    salma_buildinfo
)

if(${SALMA_WITH_CUDA})
    target_link_libraries(salma_slam PRIVATE salma_stereomatcher)
    target_compile_definitions(salma_slam PRIVATE SALMA_WITH_CUDA)
elseif(${SALMA_OPENCV_HAS_CUDA})
    target_link_libraries(salma_slam PRIVATE opencv_cudastereo opencv_cudaimgproc)
    target_compile_definitions(salma_slam PRIVATE SALMA_OPENCV_WITH_CUDA)
endif()

target_include_directories(salma_slam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS salma_slam LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

