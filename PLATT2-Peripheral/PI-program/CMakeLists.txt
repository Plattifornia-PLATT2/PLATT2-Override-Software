cmake_minimum_required(VERSION 3.15)
project(PeripheralControl VERSION 1.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(BUILD_SHARED_LIBS OFF)
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")

# Default to /opt/rpi for Docker builds, but allow override for local builds
if(NOT DEFINED RPI_ROOT)
    set(RPI_ROOT "/opt/rpi")
endif()



file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(peripheralControl ${SOURCES})

target_include_directories(peripheralControl PRIVATE
    include
    ${RPI_ROOT}/include
    ${RPI_ROOT}/include/opencv4
)

target_link_directories(peripheralControl PRIVATE
    ${RPI_ROOT}/lib
    ${RPI_ROOT}/lib/opencv4/3rdparty
)

target_link_libraries(peripheralControl PRIVATE
    -Wl,--start-group
    dl
    apriltag
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_videoio
    opencv_video
    opencv_calib3d
    opencv_features2d
    opencv_flann
    opencv_objdetect
    opencv_photo
    opencv_stitching
    opencv_highgui
    opencv_ml
    opencv_gapi
    tegra_hal
    kleidicv
    kleidicv_thread
    kleidicv_hal
    ade
    ittnotify
    zlib
    libjpeg-turbo
    libpng
    libjasper
    -Wl,--end-group
    pthread
    dl
)
