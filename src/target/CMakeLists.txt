
add_library(
    target
    STATIC
    target.h
    target.cpp
)

target_link_libraries(target PUBLIC opencv_core opencv_imgproc opencv_features2d nanoflann::nanoflann)
target_link_libraries(target PUBLIC ui) # TODO TO BE REMOVED, FOR DEBUG PURPOSE.
target_include_directories(target INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

