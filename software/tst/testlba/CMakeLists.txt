
add_executable(
    testlba
    main.cpp
)

target_link_libraries(
    testlba
    PUBLIC
    salma_slam
)

