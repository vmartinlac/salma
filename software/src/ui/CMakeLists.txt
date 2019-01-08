
add_library(
    salma_ui

    SHARED

    db.qrc

    MainWindow.h
    MainWindow.cpp
    Model.h
    Model.cpp

    SyncIO.h
    SyncIO.cpp

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

    ViewerWidget.h
    ViewerWidget.cpp
    ReconstructionInspectorDialog.h
    ReconstructionInspectorDialog.cpp

    KFDemoDialog.h
    KFDemoDialog.cpp
    KFDemoParametersDialog.h
    KFDemoParametersDialog.cpp
)

target_link_libraries(
    salma_ui
    PUBLIC
    salma_uicommon
    salma_calibration
    salma_video
    salma_slam
    salma_kfd
    salma_target
    Qt5::Widgets
    Qt5::Sql
    opencv_calib3d
)

target_include_directories(salma_ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS salma_ui LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

