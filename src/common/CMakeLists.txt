
add_library(
    common
    STATIC
    BuildInfo.h
    BuildInfo.cpp
)

target_include_directories(common INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_definitions(
    common PUBLIC
    BI_VERSION_MAJOR=0
    BI_VERSION_MINOR=1
    BI_VERSION_REVISION=0
    BI_BUILD_TYPE="$<CONFIG>"
    BI_CXX_COMPILER_ID="$<CXX_COMPILER_ID>"
    BI_CXX_COMPILER_VERSION="$<CXX_COMPILER_VERSION>"
)

