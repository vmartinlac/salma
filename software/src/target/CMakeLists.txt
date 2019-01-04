
add_library(
    salma_target
    SHARED
    Tracker.h
    Tracker.cpp
)

target_link_libraries(salma_target PUBLIC Eigen3::Eigen opencv_core opencv_imgproc opencv_features2d nanoflann::nanoflann)

target_include_directories(salma_target INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS salma_target LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

