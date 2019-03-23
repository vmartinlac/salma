
add_library(
    salma_video
    SHARED
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

target_link_libraries(salma_video PUBLIC opencv_core Qt5::Core)
target_link_libraries(salma_video PRIVATE Aravis opencv_videoio opencv_imgcodecs) # FFmpeg)
target_include_directories(salma_video INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
install(TARGETS salma_video LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

