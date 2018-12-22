
add_library(
    ui
    STATIC
    Project.h
    Project.cpp
    Model.h
    Model.cpp
    ProjectDialog.h
    ProjectDialog.cpp
    MainWindow.h
    MainWindow.cpp
    CameraCalibrationModel.h
    CameraCalibrationModel.cpp
    CameraCalibrationPanel.h
    CameraCalibrationPanel.cpp
    ImportCameraCalibrationDialog.h
    ImportCameraCalibrationDialog.cpp
    RigCalibrationModel.h
    RigCalibrationModel.cpp
    RigCalibrationPanel.h
    RigCalibrationPanel.cpp
    RecordingModel.h
    RecordingModel.cpp
    RecordingPanel.h
    RecordingPanel.cpp
    ReconstructionModel.h
    ReconstructionModel.cpp
    ReconstructionPanel.h
    ReconstructionPanel.cpp
)

target_link_libraries(
    ui
    PUBLIC
    buildinfo
    calibration
    uicommon
    Qt5::Widgets
    Qt5::Sql
)

target_include_directories(ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

