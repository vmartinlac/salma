
add_library(
    uivisualization
    STATIC
    ExportPointCloudDialog.cpp
    ExportPointCloudDialog.h
    OpenReconstructionDialog.cpp
    OpenReconstructionDialog.h
    MainWindow.cpp
    MainWindow.h
    VisualizationSettingsDialog.cpp
    VisualizationSettingsDialog.h
    ViewerWidget.cpp
    ViewerWidget.h
)

target_link_libraries(
    uivisualization
    PUBLIC
    slamcore
    buildinfo
    uicommon
    video
    Qt5::Widgets
    calibration
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    openscenegraph
    Sophus::Sophus
    Threads::Threads
)

target_include_directories(uivisualization INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

