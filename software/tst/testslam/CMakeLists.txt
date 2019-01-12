
add_executable(
    testslam
    main.cpp
)

target_link_libraries(
    testslam
    PUBLIC
    salma_slam
)

