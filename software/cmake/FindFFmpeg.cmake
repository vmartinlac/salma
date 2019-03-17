
pkg_search_module(FFmpeg REQUIRED libavcodec libavdevice libavfilter libavformat libavutil libswresample libswscale)

add_library(FFmpeg INTERFACE)
target_include_directories(FFmpeg INTERFACE ${FFmpeg_INCLUDE_DIRS})
target_link_libraries(FFmpeg INTERFACE ${FFmpeg_LDFLAGS})

