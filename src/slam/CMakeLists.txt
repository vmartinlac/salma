
add_library(
    slam
    STATIC
    SLAMEngine.h
    SLAMEngine.cpp
    DefaultSLAMEngine.h
    DefaultSLAMEngine.cpp
    SLAMThread.h
    SLAMThread.cpp
)

target_link_libraries(slam PUBLIC camera Qt5::Core Vimba opencv_core opencv_imgproc opencv_features2d opencv_xfeatures2d OpenSceneGraph Eigen3::Eigen)
target_include_directories(slam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

