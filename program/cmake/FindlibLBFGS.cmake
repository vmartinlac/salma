include(FindPackageHandleStandardArgs)

if(NOT DEFINED LIBLBFGS_ROOT)
   message(FATAL_ERROR "Please set LIBLBFGS to libLBFGS root directory!")
endif()

find_library(LIBLBFGS_LIBRARY lbfgs PATHS "${LIBLBFGS_ROOT}/lib")
find_path(LIBLBFGS_INCLUDE_DIR "lbfgs.h" PATHS "${LIBLBFGS_ROOT}/include")

find_package_handle_standard_args( libLBFGS DEFAULT_MSG LIBLBFGS_INCLUDE_DIR LIBLBFGS_LIBRARY)

add_library(libLBFGS INTERFACE)
target_include_directories(libLBFGS INTERFACE ${LIBLBFGS_INCLUDE_DIR})
target_link_libraries(libLBFGS INTERFACE ${LIBLBFGS_LIBRARY})

