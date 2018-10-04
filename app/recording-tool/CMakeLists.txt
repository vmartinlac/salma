
add_executable(
    recording-tool
    WIN32
    main.cpp
    MainWindow.h
    MainWindow.cpp
    ParametersDialog.h
    ParametersDialog.cpp
    InformationWidget.h
    InformationWidget.cpp
    RecordingThread.h
    RecordingThread.cpp
    VideoWidget.h
    VideoWidget.cpp
    Parameters.h
    Output.h
    Port.h
    Port.cpp
)

target_link_libraries(recording-tool PUBLIC camera Qt5::Core Qt5::Widgets opencv_core opencv_imgproc opencv_imgcodecs)

