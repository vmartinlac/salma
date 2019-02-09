
add_library(
    salma_uicameracalibration

    STATIC

    CameraCalibrationOperation.cpp
    CameraCalibrationOperation.h
    CameraCalibrationPanel.cpp
    CameraCalibrationPanel.h
    ManualCameraCalibrationView.cpp
    ManualCameraCalibrationView.h
    ManualCameraCalibrationDialog.cpp
    ManualCameraCalibrationDialog.h
    ManualCameraCalibrationParameters.cpp
    ManualCameraCalibrationParameters.h
    NewCameraCalibrationDialog.cpp
    NewCameraCalibrationDialog.h
    NewManualCameraCalibrationDialog.cpp
    NewManualCameraCalibrationDialog.h
)

target_link_libraries(
    salma_uicameracalibration
    PUBLIC
    salma_uicommon
    salma_target
    opencv_calib3d
)

target_include_directories(salma_uicameracalibration INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

#install(TARGETS salma_uicameracalibration LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

