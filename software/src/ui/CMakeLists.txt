
add_library(
    ui
    STATIC

    MainWindow.h
    MainWindow.cpp
    VideoWidget.h
    VideoWidget.cpp
    StatsWidget.h
    StatsWidget.cpp
    Model.h
    Model.cpp

    SyncIO.h
    SyncIO.cpp

    CameraList.h
    CameraList.cpp
    TargetScaleWidget.h
    TargetScaleWidget.cpp
    CameraCalibrationListWidget.h
    CameraCalibrationListWidget.cpp
    RigCalibrationListWidget.h
    RigCalibrationListWidget.cpp
    FrameRateWidget.h
    FrameRateWidget.cpp

    Project.h
    Project.cpp
    ProjectDialog.h
    ProjectDialog.cpp

    Operation.cpp
    Operation.h
    OperationThread.cpp
    OperationThread.h
    NewOperationDialog.h
    NewOperationDialog.cpp
    OperationDialog.h
    OperationDialog.cpp

    CameraCalibrationModel.h
    CameraCalibrationModel.cpp
    CameraCalibrationPanel.h
    CameraCalibrationPanel.cpp
    CameraCalibrationList.h
    NewCameraCalibrationDialog.h
    NewCameraCalibrationDialog.cpp
    CameraCalibrationOperation.h
    CameraCalibrationOperation.cpp

    RigCalibrationModel.h
    RigCalibrationModel.cpp
    RigCalibrationPanel.h
    RigCalibrationPanel.cpp
    RigCalibrationList.h
    NewRigCalibrationDialog.h
    NewRigCalibrationDialog.cpp
    RigCalibrationOperation.h
    RigCalibrationOperation.cpp

    RecordingModel.h
    RecordingModel.cpp
    RecordingPanel.h
    RecordingPanel.cpp
    RecordingList.h
    NewStereoRecordingDialog.h
    NewStereoRecordingDialog.cpp
    RecordingOperation.h
    RecordingOperation.cpp

    ReconstructionModel.h
    ReconstructionModel.cpp
    ReconstructionPanel.h
    ReconstructionPanel.cpp
    ReconstructionList.h
)

target_link_libraries(
    ui
    PUBLIC
    buildinfo
    calibration
    uicommon
    video
    Qt5::Widgets
    Qt5::Sql
    opencv_calib3d
)

target_include_directories(ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

