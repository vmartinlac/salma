
add_library(
    ui
    STATIC
    AboutDialog.cpp
    AboutDialog.h
    MainWindow.cpp
    MainWindow.h
    Port.cpp
    Port.h
    StatsWidget.cpp
    StatsWidget.h
    VideoWidget.cpp
    VideoWidget.h
    Operation.h
    Operation.cpp
    OperationThread.h
    OperationThread.cpp
    OperationDialog.cpp
    OperationDialog.h
    OperationParametersWidget.cpp
    OperationParametersWidget.h
    CameraCalibrationOperation.cpp
    CameraCalibrationOperation.h
    CameraCalibrationParametersWidget.cpp
    CameraCalibrationParametersWidget.h
    StereoRigCalibrationOperation.cpp
    StereoRigCalibrationOperation.h
    StereoRigCalibrationParametersWidget.cpp
    StereoRigCalibrationParametersWidget.h
    MonoRecordingOperation.cpp
    MonoRecordingOperation.h
    MonoRecordingParametersWidget.cpp
    MonoRecordingParametersWidget.h
    StereoRecordingOperation.cpp
    StereoRecordingOperation.h
    StereoRecordingParametersWidget.cpp
    StereoRecordingParametersWidget.h
)

target_link_libraries(
    ui PUBLIC
    buildinfo
    video
    target
    Qt5::Widgets
    calibration
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_calib3d
    Sophus::Sophus
    Threads::Threads
)

target_include_directories(ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

