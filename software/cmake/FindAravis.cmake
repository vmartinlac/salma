
pkg_search_module(LIBARAVIS REQUIRED aravis-0.6)
pkg_search_module(LIBGLIB2 REQUIRED glib-2.0)

add_library(Aravis INTERFACE)
target_include_directories(Aravis INTERFACE ${LIBARAVIS_INCLUDE_DIRS} ${LIBGLIB2_INCLUDE_DIRS})
target_link_libraries(Aravis INTERFACE ${LIBARAVIS_LDFLAGS} ${LIBGLIB2_LDFLAGS})

