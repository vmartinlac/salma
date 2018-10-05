
add_library(
    ui
    STATIC
    AboutDialog.h
    AboutDialog.cpp
    Port.h
    Port.cpp
    RecordingMainWindow.h
    RecordingMainWindow.cpp
    RecordingParametersDialog.h
    RecordingParametersDialog.cpp
    RecordingInformationWidget.h
    RecordingInformationWidget.cpp
    RecordingThread.h
    RecordingThread.cpp
    RecordingVideoWidget.h
    RecordingVideoWidget.cpp
    RecordingParameters.h
    RecordingOutput.h
    CameraCalibrationWindow.h
    CameraCalibrationWindow.cpp
)

target_link_libraries(ui PUBLIC common camera Qt5::Widgets OpenSceneGraph opencv_core opencv_imgproc opencv_imgcodecs)
target_include_directories(ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

