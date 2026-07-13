cmake_minimum_required(VERSION 3.15)
project(PeripheralControl VERSION 1.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(BUILD_SHARED_LIBS OFF)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(peripheralControl ${SOURCES})

target_include_directories(peripheralControl PRIVATE
    include
    /opt/rpi/include/opencv4
    /opt/rpi/include
)

target_link_directories(peripheralControl PRIVATE
    /opt/rpi/lib
    /opt/rpi/lib/opencv4/3rdparty
)

target_link_libraries(peripheralControl PRIVATE
    -Wl,--start-group
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