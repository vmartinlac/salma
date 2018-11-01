find_package(PkgConfig REQUIRED)

pkg_search_module(ARAVIS REQUIRED aravis-0.6)

add_library(Aravis INTERFACE)
target_include_directories(Aravis INTERFACE ${ARAVIS_INCLUDE_DIRS})
target_link_libraries(Aravis INTERFACE ${ARAVIS_LIBRARIES})

