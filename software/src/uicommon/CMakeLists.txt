
add_library(
    salma_uicommon

    STATIC

    AboutDialog.cpp
    AboutDialog.h
    CameraCalibrationList.h
    CameraCalibrationListWidget.cpp
    CameraCalibrationListWidget.h
    CameraCalibrationModel.cpp
    CameraCalibrationModel.h
    ReconstructionModel.cpp
    ReconstructionModel.h
    RigCalibrationModel.cpp
    RigCalibrationModel.h
    RecordingModel.cpp
    RecordingModel.h
    CameraList.cpp
    CameraList.h
    db.qrc
    FrameRateWidget.cpp
    FrameRateWidget.h
    Model.cpp
    Model.h
    NewOperationDialog.cpp
    NewOperationDialog.h
    Operation.cpp
    OperationDialog.cpp
    OperationDialog.h
    Operation.h
    OperationProgressDialog.cpp
    OperationProgressDialog.h
    OperationThread.cpp
    OperationThread.h
    PathWidget.cpp
    PathWidget.h
    Port.cpp
    Port.h
    Project.cpp
    Project.h
    RecordingList.h
    RecordingListWidget.cpp
    RecordingListWidget.h
    ReconstructionList.h
    RigCalibrationList.h
    RigCalibrationListWidget.cpp
    RigCalibrationListWidget.h
    StatsWidget.cpp
    StatsWidget.h
    TargetScaleWidget.cpp
    TargetScaleWidget.h
    VideoWidget.cpp
    VideoWidget.h
    ViewerWidgetBase.cpp
    ViewerWidgetBase.h
)

target_link_libraries(
    salma_uicommon
    PUBLIC
    salma_buildinfo
    salma_video
    salma_slam
    salma_calibration
    openscenegraph
    Qt5::Widgets
    Qt5::Sql
    Sophus::Sophus
)

target_include_directories(salma_uicommon INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

#install(TARGETS salma_uicommon LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

