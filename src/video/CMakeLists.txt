
add_library(
    video
    STATIC
    Camera.h
    Camera.cpp
    VimbaCamera.h
    VimbaCamera.cpp
    OpenCVCamera.h
    OpenCVCamera.cpp
    Image.h
    Image.cpp
    VideoSystem.h
    VideoSystem.cpp
)

target_link_libraries(video PUBLIC Vimba opencv_core opencv_videoio opencv_imgproc)
target_include_directories(video INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

