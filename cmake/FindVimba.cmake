include(FindPackageHandleStandardArgs)

find_package_handle_standard_args( Vimba DEFAULT_MSG VIMBA_C_LIBRARY VIMBA_CPP_LIBRARY VIMBA_INCLUDE_DIR )

set( VIMBA_INCLUDE_DIRS "${VIMBA_INCLUDE_DIR}")
set( VIMBA_LIBRARIES "${VIMBA_C_LIBRARY}" "${VIMBA_CPP_LIBRARY}" )

add_library(Vimba INTERFACE)
target_include_directories(Vimba INTERFACE ${VIMBA_INCLUDE_DIRS})
target_link_libraries(Vimba INTERFACE ${VIMBA_LIBRARIES})

