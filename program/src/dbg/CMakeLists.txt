
add_library(
    dbg
    STATIC
    Debug.h
    Debug.cpp
)

target_link_libraries(
    dbg
    PUBLIC
    Qt5::Widgets
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
)

target_include_directories(dbg INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

