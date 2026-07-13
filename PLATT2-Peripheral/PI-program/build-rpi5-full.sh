#!/bin/bash
# Two-step build: pre-built libs + app compilation
# Usage: ./build-rpi5-full.sh [output-directory]

LIBS_IMAGE="rpi5-libs:latest"
APP_IMAGE="peripheralcontrol:build"
OUTPUT_DIR="./peripheralControl_rpi5"

if [ -n "$1" ]; then
    OUTPUT_DIR="$1"
fi

echo ""
echo "======================================================"
echo "Raspberry Pi 5 Cross-Compilation Build System"
echo "======================================================"
echo ""
echo "Libs Image:  $LIBS_IMAGE"
echo "App Image:   $APP_IMAGE"
echo "Output:      $OUTPUT_DIR"
echo ""

# Step 1: Build pre-built libraries
echo "[STEP 1] Building pre-built libraries image..."
echo ""
docker build -f Dockerfile.rpi5-libs -t $LIBS_IMAGE .
if [ $? -ne 0 ]; then
    echo "✗ Build failed: rpi5-libs"
    exit 1
fi
echo "✓ Libraries image built successfully"
echo ""

# Step 2: Build app using the libraries
echo "[STEP 2] Building application image..."
echo ""
docker build -f Dockerfile -t $APP_IMAGE .
if [ $? -ne 0 ]; then
    echo "✗ Build failed: app"
    exit 1
fi
echo "✓ Application image built successfully"
echo ""

# Step 3: Extract binary
echo "[STEP 3] Extracting binary..."
echo ""
mkdir -p "$OUTPUT_DIR"

docker create --name rpi5-extract $APP_IMAGE /bin/sh > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "✗ Container creation failed"
    exit 1
fi

docker cp rpi5-extract:/peripheralControl "$OUTPUT_DIR/peripheralControl" > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "✗ Extraction failed"
    docker rm rpi5-extract > /dev/null 2>&1
    exit 1
fi

docker rm rpi5-extract > /dev/null 2>&1

echo "✓ Binary extracted successfully"
echo ""

if [ -f "$OUTPUT_DIR/peripheralControl" ]; then
    echo "======================================================"
    echo "BUILD SUCCESSFUL!"
    echo "======================================================"
    echo ""
    echo "Binary Details:"
    echo "  Path: $OUTPUT_DIR/peripheralControl"
    size=$(stat -c%s "$OUTPUT_DIR/peripheralControl" 2>/dev/null || stat -f%z "$OUTPUT_DIR/peripheralControl")
    echo "  Size: $size bytes"
    echo ""
else
    echo "✗ Build failed - binary not found"
    exit 1
fi
