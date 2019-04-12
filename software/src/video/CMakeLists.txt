
add_library(
    salma_video
    SHARED
    ArduinoTrigger.cpp
    ArduinoTrigger.h
    ExternalTrigger.cpp
    ExternalTrigger.h
    Image.cpp
    Image.h
    GenICamVideoSource.cpp
    GenICamVideoSource.h
    VideoSource.cpp
    VideoSource.h
    VideoSystemImpl.cpp
    VideoSystemImpl.h
    VideoSystem.cpp
    VideoSystem.h
    RecordingHeader.cpp
    RecordingHeader.h
    RecordingReader.cpp
    RecordingReader.h
    MockCamera.h
    MockCamera.cpp
    VideoErrorCode.h
    GenICamSemaphore.h
    GenICamRig.cpp
    GenICamRig.h
    GenICamCamera.cpp
    GenICamCamera.h
    RecordingHeader.h
)

target_link_libraries(salma_video PUBLIC opencv_core Qt5::Core)
target_link_libraries(salma_video PRIVATE Aravis TurboJPEG)
target_include_directories(salma_video INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
install(TARGETS salma_video LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

