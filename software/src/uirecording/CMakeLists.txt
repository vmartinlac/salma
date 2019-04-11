
add_library(
    salma_uirecording

    STATIC

    SyncIO.cpp
    SyncIO.h
    NewRecordingDialog.cpp
    NewRecordingDialog.h
    RecordingOperation.cpp
    RecordingOperation.h
    RecordingPanel.cpp
    RecordingPanel.h
    RecordingPlayerDialog.cpp
    RecordingPlayerDialog.h
)

target_link_libraries( salma_uirecording PUBLIC salma_uicommon Qt5::Widgets opencv_calib3d )
target_link_libraries( salma_uirecording PRIVATE TurboJPEG )

target_include_directories(salma_uirecording INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

#install(TARGETS salma_uirecording LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

