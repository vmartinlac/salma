
add_library(
    slamcore
    STATIC
    SLAMReconstructionDB.cpp
    SLAMReconstructionDB.h
    SLAMProject.cpp
    SLAMProject.h
    SLAMDataStructures.h
)

target_link_libraries(
    slamcore
    PUBLIC
    Qt5::Core
    Qt5::Widgets
    Qt5::Sql
    Eigen3::Eigen
    opencv_core
    video
    calibration
)

target_include_directories(slamcore INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

