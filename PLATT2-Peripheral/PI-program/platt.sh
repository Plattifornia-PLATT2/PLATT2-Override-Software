#!/bin/bash

##############################################################################
# Build Script for PLATT2 Peripheral Control - Raspberry Pi 5 (ARM64)
#
# Commands:
#   ./platt.sh build-libs      - Build/rebuild RPi5 libraries (OpenCV + AprilTag)
#   ./platt.sh build           - Fast C++ recompile using existing libs
#   ./platt.sh upload <pink|purple> - Upload binary to Raspberry Pi
#   ./platt.sh run <pink|purple>    - Run binary on Raspberry Pi
#   ./platt.sh clean           - Clean local build
#   ./platt.sh help            - Show help
##############################################################################

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Configuration
LIBS_IMAGE="platt2-rpi5-libs:latest"
BUILD_DIR="build/rpi5-local"
LIBS_EXTRACTED_DIR="build/rpi5-libs-extracted"

##############################################################################
# Build Libraries (Full Docker build - 20-30 minutes)
##############################################################################

build_libs() {
    log_info "Building RPi5 libraries (OpenCV + AprilTag)..."
    log_info "This takes 20-30 minutes on first build"
    
    docker build \
        -f Dockerfile.rpi5-libs \
        -t "${LIBS_IMAGE}" \
        .
    
    if [ $? -eq 0 ]; then
        log_success "Libraries built successfully"
        log_info "Extracting to local filesystem..."
        extract_libs
    else
        log_error "Failed to build libraries"
        exit 1
    fi
}

##############################################################################
# Extract Libraries from Docker Image
##############################################################################

extract_libs() {
    log_info "Extracting libraries from Docker image..."
    
    mkdir -p "${LIBS_EXTRACTED_DIR}"
    
    local temp_container=$(docker create "${LIBS_IMAGE}")
    docker cp "${temp_container}:/opt/rpi" "${LIBS_EXTRACTED_DIR}/"
    docker rm "${temp_container}"
    
    if [ -d "${LIBS_EXTRACTED_DIR}/rpi/include" ]; then
        log_success "Libraries extracted to ${LIBS_EXTRACTED_DIR}/rpi/"
    else
        log_error "Failed to extract libraries"
        exit 1
    fi
}

##############################################################################
# Build C++ Application (Fast - 5-30 seconds)
##############################################################################

build_app() {
    log_info "Building PeripheralControl (fast C++ compile)..."
    
    # Check if libraries are extracted
    if [ ! -d "${LIBS_EXTRACTED_DIR}/rpi/include" ]; then
        log_error "Libraries not found. Run './build-rpi5.sh libs' first"
        exit 1
    fi
    
    mkdir -p "${BUILD_DIR}"
    
    # Get absolute path to extracted libraries from current directory
    local libs_path=$(cd "${LIBS_EXTRACTED_DIR}/rpi" && pwd)
    
    cd "${BUILD_DIR}"
    
    log_info "Running CMake..."
    cmake ../.. \
        -DRPI_ROOT="${libs_path}" \
        -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
        -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
        -DCMAKE_SYSTEM_NAME=Linux \
        -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_C_FLAGS="-march=armv8-a -mtune=cortex-a76 -O3 -Wno-error=builtin-macro-redefined" \
        -DCMAKE_CXX_FLAGS="-march=armv8-a -mtune=cortex-a76 -O3 -Wno-error=builtin-macro-redefined"
    
    log_info "Compiling..."
    make -j$(nproc)
    
    cd ../..
    
    if [ -f "${BUILD_DIR}/peripheralControl" ]; then
        log_success "Binary built: ${BUILD_DIR}/peripheralControl"
    else
        log_error "Build failed"
        exit 1
    fi
}

##############################################################################
# Clean Local Build
##############################################################################

clean() {
    log_info "Cleaning local build..."
    rm -rf "${BUILD_DIR}"
    log_success "Cleaned"
}

##############################################################################
# Clean Everything
##############################################################################

clean_all() {
    log_info "Cleaning everything..."
    rm -rf "${BUILD_DIR}"
    rm -rf "${LIBS_EXTRACTED_DIR}"
    log_success "Everything cleaned"
}

##############################################################################
# Upload Binary to Raspberry Pi
##############################################################################

upload_binary() {
    local pi_name="$1"
    local pi_host
    
    case "${pi_name}" in
        pink)
            pi_host="pink"
            ;;
        purple)
            pi_host="purple"
            ;;
        "")
            log_error "Usage: $0 upload <pink|purple>"
            log_error "Example: $0 upload pink"
            log_error "Example: $0 upload purple"
            exit 1
            ;;
        *)
            log_error "Invalid target: ${pi_name}"
            log_error "Valid targets: pink, purple"
            exit 1
            ;;
    esac
    
    if [ ! -f "${BUILD_DIR}/peripheralControl" ]; then
        log_error "Binary not found at ${BUILD_DIR}/peripheralControl"
        log_error "Run '$0 build' first"
        exit 1
    fi
    
    log_info "Uploading binary to platt@${pi_host}:~"
    
    if scp "${BUILD_DIR}/peripheralControl" "platt@${pi_host}:~/"; then
        log_success "Upload complete"
        log_info "Binary ready at ~/peripheralControl on ${pi_name} (${pi_host})"
    else
        log_error "Upload failed"
        exit 1
    fi
}

##############################################################################
# Run Binary on Raspberry Pi
##############################################################################

run_binary() {
    local pi_name="$1"
    local pi_host
    
    case "${pi_name}" in
        pink)
            pi_host="pink"
            ;;
        purple)
            pi_host="purple"
            ;;
        "")
            log_error "Usage: $0 run <pink|purple>"
            log_error "Example: $0 run pink"
            log_error "Example: $0 run purple"
            exit 1
            ;;
        *)
            log_error "Invalid target: ${pi_name}"
            log_error "Valid targets: pink, purple"
            exit 1
            ;;
    esac
    
    log_info "Running peripheralControl on platt@${pi_host}"
    
    if ssh -t "platt@${pi_host}" "chmod +x ~/peripheralControl && ~/peripheralControl"; then
        log_success "Execution completed"
    else
        log_error "Execution failed"
        exit 1
    fi
}

##############################################################################
# Help
##############################################################################

show_help() {
    cat << EOF
${BLUE}PLATT2 Peripheral Control - RPi5 Build Script${NC}

Usage: $0 [COMMAND]

Commands:
    build-libs      Build/rebuild libraries (Docker, ~20-30 min first time)
    build           Fast C++ recompile (5-30 sec, incremental)
    upload <pink|purple>  Upload binary to Raspberry Pi
    run <pink|purple>     Run binary on Raspberry Pi
    clean           Clean local build only
    clean-all       Clean everything including extracted libraries
    help            Show this help

Examples:
    $0 build-libs             # Full rebuild of OpenCV + AprilTag
    $0 build                  # Fast recompile (only changed C++ files)
    $0 upload pink            # Upload to pink.local
    $0 run pink               # Run on pink.local

Workflow:
    1. First run:    $0 build-libs    (builds and extracts libraries)
    2. Then:         $0 build         (fast C++ compile)
    3. Repeat step 2 for quick iteration

Output:
    Binary: ${BUILD_DIR}/peripheralControl

Transfer to RPi5:
    scp ${BUILD_DIR}/peripheralControl pi@<ip>:~/

EOF
}

##############################################################################
# Main
##############################################################################

main() {
    local command="${1:-help}"
    
    case "${command}" in
        libs)
            build_libs
            ;;
        build)
            build_app
            ;;
        build-libs)
            build_libs
            ;;
        upload)
            upload_binary "$2"
            ;;
        run)
            run_binary "$2"
            ;;
        clean)
            clean
            ;;
        clean-all)
            clean_all
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            log_error "Unknown command: ${command}"
            show_help
            exit 1
            ;;
    esac
}

main "$@"
