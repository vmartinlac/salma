
add_executable(
    featuresdemo
    WIN32
    main.cpp
)

#target_link_libraries(featuresdemo PUBLIC Qt5::Widgets camera ui features)
target_link_libraries(featuresdemo PUBLIC features opencv_highgui)

