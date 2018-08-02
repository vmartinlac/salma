
add_executable(
    demo
    WIN32
    main.cpp
    utils.h
    utils.cpp
)

target_link_libraries(demo PUBLIC Qt5::Widgets camera ui slam Boost::boost)

