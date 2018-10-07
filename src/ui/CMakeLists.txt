
add_library(
    ui
    STATIC
    AboutDialog.h
    AboutDialog.cpp
    Port.h
    Port.cpp
    VideoWidget.h
    VideoWidget.cpp
    VideoInputDialog.h
    VideoInputDialog.cpp
    RecordingParameters.h
    RecordingParametersDialog.h
    RecordingParametersDialog.cpp
    RecordingStatsWidget.h
    RecordingStatsWidget.cpp
    RecordingThread.h
    RecordingThread.cpp
    RecordingMainWindow.h
    RecordingMainWindow.cpp
    CameraCalibrationParameters.h
    CameraCalibrationParametersDialog.h
    CameraCalibrationParametersDialog.cpp
    CameraCalibrationStatsWidget.h
    CameraCalibrationStatsWidget.cpp
    CameraCalibrationThread.h
    CameraCalibrationThread.cpp
    CameraCalibrationMainWindow.h
    CameraCalibrationMainWindow.cpp
)

target_link_libraries(
    ui PUBLIC
    common
    camera
    target
    Qt5::Widgets
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_calib3d
)

target_include_directories(ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

