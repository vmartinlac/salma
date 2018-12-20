
add_library(
    ui
    STATIC
    MainWindow.h
    MainWindow.cpp
    CameraCalibrationModel.h
    CameraCalibrationModel.cpp
    CameraCalibrationPanel.h
    CameraCalibrationPanel.cpp
    RigCalibrationPanel.h
    RigCalibrationPanel.cpp
    RecordingPanel.h
    RecordingPanel.cpp
    ReconstructionPanel.h
    ReconstructionPanel.cpp
)

target_link_libraries(
    ui
    PUBLIC
    buildinfo
    uicommon
    Qt5::Widgets
)

target_include_directories(ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

