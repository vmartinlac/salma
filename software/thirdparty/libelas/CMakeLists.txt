
add_library(
    salma_libelas
    SHARED 
    descriptor.cpp
    descriptor.h
    elas.cpp
    elas.h
    filter.cpp
    filter.h
    image.h
    main.cpp
    matrix.cpp
    matrix.h
    timer.h
    triangle.cpp
    triangle.h
    ElasIntf.h
    ElasIntf.cpp
    ElasIntfImpl.h
    ElasIntfImpl.cpp
)

target_link_libraries(salma_libelas PUBLIC opencv_core)
target_compile_options(salma_libelas PRIVATE "-msse3")
target_include_directories(salma_libelas INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
install(TARGETS salma_libelas LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

