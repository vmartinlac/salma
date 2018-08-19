
add_library(
    ui
    STATIC
    MainWindow.h
    MainWindow.cpp
    ViewerWidget.h
    ViewerWidget.cpp
    utils.h
    utils.cpp
    ParametersDialog.h
    ParametersDialog.cpp
    VideoWidget.h
    VideoWidget.cpp
    StatsWidget.h
    StatsWidget.cpp
    VideoInputDialog.h
    VideoInputDialog.cpp
)

target_link_libraries(ui PUBLIC slam camera Qt5::Widgets OpenSceneGraph opencv_core opencv_imgproc)
target_include_directories(ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

