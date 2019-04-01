
add_executable(
    newvid
    main.cpp
    Camera.h
    Camera.cpp
    Rig.h
    Rig.cpp
    GenICamConfig.h
    Semaphore.h
)

target_link_libraries(
    newvid
    PUBLIC
    #Vimba
    Aravis
    opencv_core
    Qt5::Core
)

