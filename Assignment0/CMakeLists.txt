cmake_minimum_required (VERSION 3.15)

include(FetchContent)
FetchContent_Declare(vcpkg
    vcpkg
    URL "https://github.com/microsoft/vcpkg/archive/master.zip")
FetchContent_MakeAvailable(vcpkg)
set(CMAKE_TOOLCHAIN_FILE ${vcpkg_SOURCE_DIR}/scripts/buildsystems/vcpkg.cmake)
project (Transformation)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

add_executable (Transformation main.cpp)
