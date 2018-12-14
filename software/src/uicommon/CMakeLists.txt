
add_library(
    uicommon
    STATIC
    PathWidget.h
    PathWidget.cpp
    AboutDialog.cpp
    AboutDialog.h
    Port.cpp
    Port.h
)

target_link_libraries(
    uicommon
    PUBLIC
    buildinfo
    target
    Qt5::Widgets
)

target_include_directories(uicommon INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

