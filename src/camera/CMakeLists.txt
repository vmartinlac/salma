
add_library(
    camera
    STATIC
    Camera.h
    Camera.cpp
    VimbaCamera.h
    VimbaCamera.cpp
    OpenCVCamera.h
    OpenCVCamera.cpp
    Image.h
    Image.cpp
)

target_link_libraries(camera PUBLIC Vimba opencv_core)
target_include_directories(camera INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

