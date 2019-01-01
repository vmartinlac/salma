
add_library(
    ui
    STATIC

    db.qrc

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
    RecordingListWidget.h
    RecordingListWidget.cpp
    FrameRateWidget.h
    FrameRateWidget.cpp

    Project.h
    Project.cpp

    Operation.cpp
    Operation.h
    OperationThread.cpp
    OperationThread.h
    OperationProgressDialog.h
    OperationProgressDialog.cpp
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
    NewMonoRecordingDialog.h
    NewMonoRecordingDialog.cpp
    NewStereoRecordingDialog.h
    NewStereoRecordingDialog.cpp
    RecordingOperation.h
    RecordingOperation.cpp
    RecordingPlayerDialog.h
    RecordingPlayerDialog.cpp

    ReconstructionModel.h
    ReconstructionModel.cpp
    ReconstructionPanel.h
    ReconstructionPanel.cpp
    ReconstructionList.h
    NewReconstructionDialog.h
    NewReconstructionDialog.cpp
    ReconstructionOperation.h
    ReconstructionOperation.cpp
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

