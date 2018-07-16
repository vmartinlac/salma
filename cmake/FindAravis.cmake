find_package(PkgConfig REQUIRED)

pkg_check_modules(glib REQUIRED glib-2.0)

include(FindPackageHandleStandardArgs)

find_path(ARAVIS_INCLUDE_DIR arv.h /usr/include/aravis-0.6)
find_library(ARAVIS_LIBRARY aravis-0.6 /usr/lib)

find_package_handle_standard_args(ARAVIS DEFAULT_MSG
  ARAVIS_INCLUDE_DIR
  ARAVIS_LIBRARY)

set(ARAVIS_INCLUDE_DIRS "${ARAVIS_INCLUDE_DIR}")
set(ARAVIS_LIBRARIES "${ARAVIS_LIBRARIES}")

add_library(Aravis INTERFACE)
target_include_directories(Aravis INTERFACE ${ARAVIS_INCLUDE_DIRS} ${glib_INCLUDE_DIRS})
target_link_libraries(Aravis INTERFACE ${ARAVIS_LIBRARIES} ${glib_LIBRARIES})
