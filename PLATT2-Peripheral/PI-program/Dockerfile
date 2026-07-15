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

COPY CMakeLists.txt ./CMakeLists.txt

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
    -DCMAKE_C_FLAGS="-Wno-error=builtin-macro-redefined" \
    -DCMAKE_CXX_FLAGS="-Wno-error=builtin-macro-redefined" \
    -DCMAKE_EXE_LINKER_FLAGS="-Wl,--allow-shlib-undefined" \
    && make -j4

# Output stage - extract binary
FROM scratch AS output
COPY --from=builder /build/build_app/peripheralControl /peripheralControl
