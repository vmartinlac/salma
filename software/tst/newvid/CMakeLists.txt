add_executable(
    newvid
    main.cpp
    Camera.h
    Camera.cpp
    Rig.h
    Rig.cpp
    GenICamConfig.h
)

target_link_libraries(
    newvid
    PUBLIC
    Aravis
    opencv_core
    Qt5::Core
)

