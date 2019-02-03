
add_library(
    salma_stereomatcher
    SHARED
    StereoMatcher.h
    StereoMatcherImpl.h
    StereoMatcherImpl.cu
    PhotometricNormalizer.h
    PhotometricNormalizer.cpp
)

target_link_libraries(
    salma_stereomatcher
    PUBLIC
    opencv_core
    opencv_calib3d)

target_include_directories(salma_stereomatcher INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS salma_stereomatcher LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

