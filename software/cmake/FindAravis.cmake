find_package(PkgConfig REQUIRED)

pkg_search_module(ARAVIS REQUIRED aravis-0.6)
pkg_search_module(GLIB REQUIRED glib-2.0)

add_library(Aravis INTERFACE)
target_include_directories(Aravis INTERFACE ${ARAVIS_INCLUDE_DIRS} ${GLIB_LIBRARIES})
target_link_libraries(Aravis INTERFACE ${ARAVIS_LDFLAGS} ${GLIB_LDFLAGS})

