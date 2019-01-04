
add_executable(
    detecttarget
    main.cpp
)

target_link_libraries(detecttarget PUBLIC salma_target Qt5::Core)

install(TARGETS detecttarget RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

