
add_library(
    video
    STATIC
    ArduinoTrigger.cpp
    ArduinoTrigger.h
    ExternalTrigger.cpp
    ExternalTrigger.h
    Image.cpp
    Image.h
    VideoSource.cpp
    VideoSource.h
    VideoSystemImpl.cpp
    VideoSystemImpl.h
    VideoSystem.cpp
    VideoSystem.h
    RecordingReader.cpp
    RecordingReader.h
    RecordingHeader.h
    MockCamera.h
    MockCamera.cpp
    VideoErrorCode.h
    GenICamRig.cpp
    GenICamRig.h
    GenICamCamera.cpp
    GenICamCamera.h
    RecordingHeader.h
)

target_link_libraries(video PUBLIC Aravis opencv_core opencv_videoio opencv_imgproc opencv_imgcodecs Qt5::Core)
target_include_directories(video INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

