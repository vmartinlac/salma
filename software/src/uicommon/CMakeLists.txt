
add_library(
    salma_uicommon

    STATIC

    AboutDialog.cpp
    AboutDialog.h
    CalibrationListWidget.cpp
    CalibrationListWidget.h
    CameraList.cpp
    CameraList.h
    FrameRateWidget.cpp
    FrameRateWidget.h
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
    RecordingListWidget.cpp
    RecordingListWidget.h
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
    salma_persistence
    openscenegraph
    Qt5::Widgets
    Qt5::Sql
)

target_include_directories(salma_uicommon INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

#install(TARGETS salma_uicommon LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

