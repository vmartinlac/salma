include(FindPackageHandleStandardArgs)

if(NOT DEFINED VIMBA_ROOT)
   message(FATAL_ERROR "Please set VIMBA_ROOT to Vimba root directory!")
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    #find_library(VIMBA_C_LIBRARY VimbaC PATHS "${VIMBA_ROOT}/VimbaC/DynamicLib/x86_64bit")
    find_library(VIMBA_CPP_LIBRARY VimbaC PATHS "${VIMBA_ROOT}/VimbaCPP/DynamicLib/x86_64bit")
else()
    #find_library(VIMBA_C_LIBRARY VimbaC PATHS "${VIMBA_ROOT}/VimbaC/DynamicLib/x86_32bit")
    find_library(VIMBA_CPP_LIBRARY VimbaC PATHS "${VIMBA_ROOT}/VimbaCPP/DynamicLib/x86_32bit")
endif()
find_path(VIMBA_INCLUDE_DIR "VimbaC/Include/VimbaC.h" PATHS "${VIMBA_ROOT}")

find_package_handle_standard_args( Vimba DEFAULT_MSG VIMBA_CPP_LIBRARY VIMBA_INCLUDE_DIR)

add_library(Vimba INTERFACE)
target_include_directories(Vimba INTERFACE ${VIMBA_INCLUDE_DIR})
target_link_libraries(Vimba INTERFACE ${VIMBA_CPP_LIBRARY})

