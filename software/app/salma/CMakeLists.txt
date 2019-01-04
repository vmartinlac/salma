
add_executable(
    salma
    WIN32
    main.cpp
)

target_link_libraries(salma PUBLIC salma_ui)

install(TARGETS salma RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

