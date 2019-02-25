
add_library(
    salma_uicalibration

    STATIC

    CalibrationPanel.cpp
    CalibrationPanel.h
    ManualCalibrationDialog.cpp
    ManualCalibrationDialog.h
    ManualCalibrationParameters.cpp
    ManualCalibrationParameters.h
    ManualCalibrationView.cpp
    ManualCalibrationView.h
    NewManualCalibrationDialog.cpp
    NewManualCalibrationDialog.h
)

target_link_libraries(
    salma_uicalibration
    PUBLIC
    salma_uicommon
    salma_target
    Qt5::Widgets
    opencv_calib3d
)

target_include_directories(salma_uicalibration INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

#install(TARGETS salma_uicalibration LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

