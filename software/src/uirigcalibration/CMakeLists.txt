
add_library(
    salma_uirigcalibration

    STATIC

    NewRigCalibrationDialog.cpp
    NewRigCalibrationDialog.h
    RigCalibrationOperation.cpp
    RigCalibrationOperation.h
    RigCalibrationPanel.cpp
    RigCalibrationPanel.h
)

target_link_libraries(
    salma_uirigcalibration
    PUBLIC
    salma_uicommon
    salma_target
    Qt5::Widgets
    opencv_calib3d
)

target_include_directories(salma_uirigcalibration INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

#install(TARGETS salma_uirigcalibration LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

