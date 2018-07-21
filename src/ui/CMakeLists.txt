
add_library(
    ui
    STATIC
    MainWindow.h
    MainWindow.cpp
    ViewerWidget.h
    ViewerWidget.cpp
)

target_link_libraries(ui PUBLIC slam camera Qt5::Widgets OpenSceneGraph)
target_include_directories(ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

