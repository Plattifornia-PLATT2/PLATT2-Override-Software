# Main Dockerfile - Uses pre-built OpenCV + AprilTag
# First build the base libs image:
#   docker build -f Dockerfile.rpi5-libs -t rpi5-libs:latest .
#
# Then build your app:
#   docker build -f Dockerfile -t rpi5-apriltag:latest .

FROM rpi5-libs:latest AS builder

WORKDIR /build
COPY src ./src
COPY include ./include

# Create CMakeLists.txt for the app
RUN cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.16)
project(peripheralControl CXX C)
set(CMAKE_CXX_STANDARD 17)

# Find AprilTag
find_package(apriltag REQUIRED CONFIG PATHS /opt/rpi/lib/apriltag/cmake NO_DEFAULT_PATH)

# Find OpenCV
find_package(OpenCV REQUIRED CONFIG PATHS /opt/rpi/lib/cmake/opencv4 NO_DEFAULT_PATH COMPONENTS core imgproc videoio imgcodecs)

add_executable(peripheralControl src/main.cpp src/ImageProssesing.cpp)

target_include_directories(peripheralControl PRIVATE 
    include 
    ${apriltag_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(peripheralControl PRIVATE 
    apriltag::apriltag
    ${OpenCV_LIBRARIES}
    m pthread
)

# Static linking
target_link_options(peripheralControl PRIVATE 
    -static-libgcc 
    -static-libstdc++
)

install(TARGETS peripheralControl DESTINATION /)
EOF

# Build the application
RUN mkdir -p build_app && cd build_app && \
    cmake .. \
    -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
    -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/opt/rpi/lib/cmake \
    -DCMAKE_MODULE_PATH=/opt/rpi/lib/cmake \
    && make -j$(nproc)

# Output stage - extract binary
FROM scratch AS output
COPY --from=builder /build/build_app/peripheralControl /peripheralControl
