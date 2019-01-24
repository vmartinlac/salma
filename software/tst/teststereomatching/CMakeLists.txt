
add_executable(
    teststereomatching
    main.cpp
)

target_link_libraries(
    teststereomatching
    PUBLIC
    salma_stereomatcher
    Eigen3::Eigen
    opencv_imgcodecs
    opencv_cudastereo
)

