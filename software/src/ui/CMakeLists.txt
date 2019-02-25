
add_library(
    salma_ui

    SHARED

    MainWindow.h
    MainWindow.cpp
)

target_link_libraries(
    salma_ui
    PUBLIC
    salma_uirecording
    salma_uicalibration
    salma_uireconstruction
    Qt5::Widgets
    Qt5::Sql
    opencv_calib3d
)

target_include_directories(salma_ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS salma_ui LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

