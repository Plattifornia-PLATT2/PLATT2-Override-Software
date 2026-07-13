#!/bin/bash
# Quick rebuild app only (assumes rpi5-libs:latest already built)
# Usage: ./rebuild-app.sh [output-directory]

OUTPUT_DIR="./peripheralControl_rpi5"

if [ -n "$1" ]; then
    OUTPUT_DIR="$1"
fi

echo ""
echo "Rebuilding application"
echo "Output: $OUTPUT_DIR"
echo ""

# Build app
docker build -f Dockerfile -t peripheralcontrol:build .
if [ $? -ne 0 ]; then
    echo "✗ Build failed"
    exit 1
fi

# Extract binary
mkdir -p "$OUTPUT_DIR"

docker create --name rpi5-extract peripheralcontrol:build /bin/sh > /dev/null 2>&1
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

if [ -f "$OUTPUT_DIR/peripheralControl" ]; then
    echo ""
    echo "======================================================"
    echo "✓ REBUILD SUCCESSFUL!"
    echo "======================================================"
    echo ""
    echo "Binary: $OUTPUT_DIR/peripheralControl"
    size=$(stat -c%s "$OUTPUT_DIR/peripheralControl" 2>/dev/null || stat -f%z "$OUTPUT_DIR/peripheralControl")
    echo "File size: $size bytes"
    echo ""
else
    echo "✗ Rebuild failed"
    exit 1
fi
