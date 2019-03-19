
add_library(
    salma_uirecording

    STATIC

    NewMonoRecordingDialog.cpp
    NewMonoRecordingDialog.h
    NewStereoRecordingDialog.cpp
    NewStereoRecordingDialog.h
    RecordingOperation.cpp
    RecordingOperation.h
    RecordingPanel.cpp
    RecordingPanel.h
    RecordingPlayerDialog.cpp
    RecordingPlayerDialog.h
    SyncIO.cpp
    SyncIO.h
)

target_link_libraries(
    salma_uirecording
    PUBLIC
    salma_uicommon
    Qt5::Widgets
    opencv_calib3d
)

target_link_libraries(
    salma_uirecording
    PRIVATE
    FFmpeg
)

target_include_directories(salma_uirecording INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

#install(TARGETS salma_uirecording LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

