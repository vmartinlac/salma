
pkg_search_module(LIBTURBOJPEG REQUIRED libturbojpeg)

add_library(TurboJPEG INTERFACE)
target_include_directories(TurboJPEG INTERFACE ${LIBTURBOJPEG_INCLUDE_DIRS})
target_link_libraries(TurboJPEG INTERFACE ${LIBTURBOJPEG_LDFLAGS})

