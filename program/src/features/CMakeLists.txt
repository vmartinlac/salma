
add_library(
    features
    STATIC
    FeatureDetector.h
    FeatureDetector.cpp

)

target_link_libraries(
    features
    PUBLIC
    opencv_core
    opencv_imgproc
    opencv_features2d)

target_include_directories(features INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

