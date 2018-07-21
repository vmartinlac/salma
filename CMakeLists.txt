cmake_minimum_required(VERSION 3.1)

project(slam CXX)

include(GNUInstallDirs)
include(GenerateExportHeader)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(Qt5 COMPONENTS OpenGL Core Widgets Gui PrintSupport Test Multimedia REQUIRED)
find_package(Vimba REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OSG REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
#find_package(Aravis REQUIRED)
#find_package(Boost 1.60.0 REQUIRED COMPONENTS math_tr1 random program_options)
#find_package(OpenMP REQUIRED)

add_subdirectory(src)

