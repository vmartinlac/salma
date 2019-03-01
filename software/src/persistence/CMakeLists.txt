
add_library(
    salma_persistence

    STATIC

    CalibrationList.h
    CalibrationModel.cpp
    CalibrationModel.h
    Model.cpp
    Model.h
    Project.cpp
    Project.h
    ReconstructionList.h
    ReconstructionModel.cpp
    ReconstructionModel.h
    RecordingList.h
    RecordingModel.cpp
    RecordingModel.h
)

target_link_libraries(
    salma_persistence
    PUBLIC
    salma_video
    salma_slam
    salma_calibration
    Qt5::Widgets
    Qt5::Sql
)

target_include_directories(salma_persistence INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

#install(TARGETS salma_persistence LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

