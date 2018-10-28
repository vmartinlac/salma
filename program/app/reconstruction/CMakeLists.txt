
add_executable(
    reconstruction
    main.cpp
    SLAMSystem.h
    SLAMSystem.cpp
)

target_link_libraries(reconstruction PUBLIC features buildinfo slam)

