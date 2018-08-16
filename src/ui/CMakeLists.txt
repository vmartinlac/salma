
add_library(
    ui
    STATIC
    MainWindow.h
    MainWindow.cpp
    ViewerWidget.h
    ViewerWidget.cpp
    utils.h
    utils.cpp
    SLAMParametersDialog.h
    SLAMParametersDialog.cpp
    VideoWidget.h
    VideoWidget.cpp
    StatsWidget.h
    StatsWidget.cpp
)

target_link_libraries(ui PUBLIC slam camera Qt5::Widgets OpenSceneGraph opencv_core opencv_imgproc)
target_include_directories(ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

