

add_library(
    salma_uicommon
    SHARED

    CameraList.h
    CameraList.cpp
    AboutDialog.h
    AboutDialog.cpp
    PathWidget.h
    PathWidget.cpp
    Port.h
    Port.cpp
    ViewerWidgetBase.h
    ViewerWidgetBase.cpp
    VideoWidget.h
    VideoWidget.cpp
    StatsWidget.h
    StatsWidget.cpp
)

target_link_libraries(
    salma_uicommon
    PUBLIC
    salma_buildinfo
    salma_video
    openscenegraph
    Qt5::Widgets
)

target_include_directories(salma_uicommon INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS salma_uicommon LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

