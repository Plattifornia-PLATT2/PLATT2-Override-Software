cmake_minimum_required(VERSION 3.15)
project(PeripheralControl VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(BUILD_SHARED_LIBS OFF)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(peripheralControl ${SOURCES})
target_include_directories(peripheralControl PRIVATE include)

set(apriltag_DIR "${CMAKE_SOURCE_DIR}/lib/apriltag/install/lib/apriltag/cmake")
find_package(apriltag REQUIRED)
target_link_libraries(peripheralControl PRIVATE apriltag::apriltag)
