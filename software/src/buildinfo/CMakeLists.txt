
add_library(
    salma_buildinfo
    SHARED
    BuildInfo.h
    BuildInfo.cpp
)

target_include_directories(salma_buildinfo INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_definitions(
    salma_buildinfo
    PUBLIC
    BI_VERSION_MAJOR=0
    BI_VERSION_MINOR=1
    BI_VERSION_REVISION=0
    #BI_BUILD_TYPE="$<CONFIG>"
    BI_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
    BI_CXX_COMPILER_ID="$<CXX_COMPILER_ID>"
    BI_CXX_COMPILER_VERSION="$<CXX_COMPILER_VERSION>"
)

install(TARGETS salma_buildinfo LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

