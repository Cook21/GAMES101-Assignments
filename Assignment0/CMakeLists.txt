cmake_minimum_required (VERSION 3.20)
set(CMAKE_CXX_STANDARD 17)
include(FetchContent)
FetchContent_Declare(vcpkg
    vcpkg
    URL "https://github.com/microsoft/vcpkg/archive/master.zip")
FetchContent_MakeAvailable(vcpkg)
set(CMAKE_TOOLCHAIN_FILE ${vcpkg_SOURCE_DIR}/scripts/buildsystems/vcpkg.cmake
  CACHE STRING "Vcpkg toolchain file")
project (Transformation)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
add_executable (Transformation main.cpp)
find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(Transformation PRIVATE Eigen3::Eigen)
