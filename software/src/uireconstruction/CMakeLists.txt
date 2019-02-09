
add_library(
    salma_uireconstruction

    STATIC

    NewReconstructionDialog.cpp
    NewReconstructionDialog.h
    ReconstructionInspectorDialog.cpp
    ReconstructionInspectorDialog.h
    ReconstructionOperation.cpp
    ReconstructionOperation.h
    ReconstructionPanel.cpp
    ReconstructionPanel.h
    ViewerWidget.cpp
    ViewerWidget.h
)

target_link_libraries(
    salma_uireconstruction
    PUBLIC
    salma_uicommon
    Qt5::Widgets
    Qt5::Sql
    opencv_calib3d
)

target_include_directories(salma_uireconstruction INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

#install(TARGETS salma_uireconstruction LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

