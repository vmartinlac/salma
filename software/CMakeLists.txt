cmake_minimum_required(VERSION 3.1)

project(slam)

option(SALMA_WITH_CUDA "Whether to use CUDA" OFF)

enable_language(CXX)
if(${SALMA_WITH_CUDA})
    enable_language(CUDA)
endif()

include(GNUInstallDirs)
include(GenerateExportHeader)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(PkgConfig REQUIRED)
find_package(Aravis REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(g2o REQUIRED)
find_package(nanoflann REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OSG REQUIRED)
find_package(Qt5 COMPONENTS OpenGL Sql Core Widgets Gui PrintSupport Test REQUIRED)
find_package(Sophus REQUIRED)
#find_package(Threads REQUIRED)
#find_package(FFmpeg REQUIRED)
#find_package(Threads REQUIRED)

add_subdirectory(src)
add_subdirectory(app)
add_subdirectory(tst)
add_subdirectory(thirdparty)

# Old stuff:

#find_package(libLBFGS REQUIRED)
#find_package(Vimba REQUIRED)
#find_package(OSG REQUIRED)
#find_package(Curses REQUIRED)
#find_package(Boost 1.60.0 REQUIRED COMPONENTS math_tr1 random program_options)

# create target for Curses
#add_library(Curses INTERFACE)
#target_include_directories(Curses INTERFACE ${CURSES_INCLUDE_DIRS})
#target_link_libraries(Curses INTERFACE ${CURSES_LIBRARIES})

