
pkg_search_module(FFMPEG REQUIRED libavformat libavcodec libavdevice libavfilter libavutil libswresample libswscale)

add_library(FFmpeg INTERFACE)
target_include_directories(FFmpeg INTERFACE ${FFMPEG_INCLUDE_DIRS})
target_link_libraries(FFmpeg INTERFACE ${FFMPEG_LDFLAGS})

