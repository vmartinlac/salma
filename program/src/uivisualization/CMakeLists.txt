
add_library(
    uivisualization
    STATIC
    MainWindow.cpp
    MainWindow.h
    ViewerWidget.cpp
    ViewerWidget.h
)

target_link_libraries(
    uivisualization
    PUBLIC
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

