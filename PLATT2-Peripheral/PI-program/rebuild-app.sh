#!/bin/bash
# Quick rebuild app only (assumes rpi5-libs:latest already built)
# Usage: ./rebuild-app.sh [output-directory]

set -e

OUTPUT_DIR="${1:-.}/peripheralControl_rpi5"

echo ""
echo "Rebuilding application only..."
echo "Output: $OUTPUT_DIR"
echo ""

# Build app
docker build -f Dockerfile -t peripheralControl:build .

# Extract binary
mkdir -p "$OUTPUT_DIR"

docker create --name rpi5-extract peripheralControl:build /bin/sh
docker cp rpi5-extract:/peripheralControl "$OUTPUT_DIR/peripheralControl"
docker rm rpi5-extract

if [ -f "$OUTPUT_DIR/peripheralControl" ]; then
    echo ""
    echo "======================================================"
    echo "✓ REBUILD SUCCESSFUL!"
    echo "======================================================"
    echo ""
    echo "Binary: $OUTPUT_DIR/peripheralControl"
    ls -lh "$OUTPUT_DIR/peripheralControl"
    echo ""
else
    echo "✗ Rebuild failed"
    exit 1
fi
