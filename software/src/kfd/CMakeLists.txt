
add_library(
    salma_kfd
    SHARED
    KFDFilter.h
    KFDFilter.cpp
    KFDEngine.h
    KFDEngine.cpp
    #MainWindow.h
    #MainWindow.cpp
    KFDViewer.h
    KFDViewer.cpp
    KFDPose.h
)

target_link_libraries(
    salma_kfd
    PUBLIC
    Qt5::Core
    Qt5::Widgets
    Eigen3::Eigen
    opencv_imgproc
    opencv_calib3d
    salma_video
    salma_uicommon
    salma_calibration
    salma_target
)

target_include_directories(salma_kfd INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS salma_kfd LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

