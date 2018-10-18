
add_library(
    video
    STATIC
    Image.h
    Image.cpp
    Camera.h
    Camera.cpp
    Trigger.h
    Trigger.cpp
    VideoSource.h
    VideoSource.cpp
    AssembledVideoSource.h
    AssembledVideoSource.cpp
    GeneralVideoSource.h
    GeneralVideoSource.cpp
    VideoSystem.h
    VideoSystem.cpp
    MockCamera.h
    MockCamera.cpp
    AvtCamera.h
    AvtCamera.cpp
    ArduinoTrigger.h
    ArduinoTrigger.cpp
)

target_link_libraries(video PUBLIC Vimba opencv_core opencv_videoio opencv_imgproc)
target_include_directories(video INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

