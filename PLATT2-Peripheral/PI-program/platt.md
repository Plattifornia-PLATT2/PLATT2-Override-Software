# Build Script for PLATT2 Peripheral Control - Raspberry Pi 5

This script handles cross-compilation for Raspberry Pi 5 (ARM64) using Docker and local CMake.

## Quick Start

```bash
# First time: build libraries (Docker, ~20-30 minutes)
./platt.sh build-libs

# Fast recompile C++ (5-30 seconds, incremental)
./platt.sh build

# Repeat the build command for each code change
./platt.sh build
```

## Commands

### `build-libs` - Build/Rebuild Libraries
```bash
./platt.sh build-libs
```
- Builds Docker image with OpenCV 4.x + AprilTag for ARM64
- Optimized for Raspberry Pi 5 (cortex-a76 CPU tuning)
- Extracts libraries to `build/rpi5-libs-extracted/rpi/`
- Takes 20-30 minutes first run, cached thereafter

Use this when:
- First-time setup
- Updating library versions
- Changing CMake library configuration

### `build` - Fast C++ Recompile
```bash
./platt.sh build
```
- Uses pre-built libraries
- Runs local CMake + make for C++ compilation
- Only recompiles changed files
- Takes 5-30 seconds (incremental)

Use this for:
- Rapid iteration during development
- Testing code changes
- Quick debugging cycles

### `clean` - Clean Local Build
```bash
./platt.sh clean
```
- Removes compiled binary and build artifacts
- Keeps extracted libraries intact
- Keeps Docker image cached

### `clean-all` - Full Cleanup
```bash
./platt.sh clean-all
```
- Removes all build artifacts
- Removes extracted libraries
- Keeps Docker image (can be rebuilt with `libs`)

### `upload <pink|purple>` - Upload to Raspberry Pi
```bash
./platt.sh upload pink
./platt.sh upload purple
```

Uploads the compiled binary to the specified Raspberry Pi.
- **Targets:** `pink` (→ pink.local) or `purple` (→ purple.local)
- **User:** `platt`
- **Destination:** `~/peripheralControl`

After upload, SSH into the Pi and run:
```bash
ssh platt@pink.local
chmod +x ~/peripheralControl
./peripheralControl
```

### `run <pink|purple>` - Run Binary on Raspberry Pi
```bash
./platt.sh run pink
./platt.sh run purple
```

Connects to the specified Raspberry Pi via SSH and executes the binary.
- **Targets:** `pink` (→ pink.local) or `purple` (→ purple.local)
- **User:** `platt`
- Automatically makes the binary executable and runs it

### `help` - Show Help
```bash
./platt.sh help
```

## Workflow

### Initial Setup
```bash
./platt.sh build-libs   # Build libraries once (~30 min)
./platt.sh build        # Compile application
```

### During Development
```bash
./platt.sh build        # Edit code, compile
./platt.sh build        # Edit code, compile (faster, incremental)
./platt.sh build        # Repeat...
```

### Update Libraries
```bash
./platt.sh build-libs   # Rebuild libraries with new config
./platt.sh build        # Recompile application
```

## Build Artifacts

- **Binary**: `build/rpi5-local/peripheralControl` (ARM64 static executable)
- **Libraries**: `build/rpi5-libs-extracted/rpi/` (OpenCV + AprilTag headers/libs)

## Transfer to Raspberry Pi 5

Use the built-in upload command:

```bash
./platt.sh upload pink    # Upload to pink.local
./platt.sh upload purple  # Upload to purple.local
```

This uploads to `~/peripheralControl` on the Pi as user `platt`.

### Manual Transfer (Alternative)

If you prefer to use `scp` manually:

```bash
scp build/rpi5-local/peripheralControl platt@pink.local:~/
scp build/rpi5-local/peripheralControl platt@purple.local:~/
```

## Build Performance

| Stage | Time |
|-------|------|
| First `libs` build | 20-30 minutes |
| Cached `libs` (no changes) | N/A (use cached image) |
| First `app` build | 30 seconds |
| Incremental `app` build | 5-10 seconds |

## Docker Images

- **`platt2-rpi5-libs:latest`** — Pre-built OpenCV + AprilTag for ARM64
  - Built by `./build-rpi5.sh libs`
  - Reused across multiple app builds
  - Can be pushed to a registry for CI/CD

## Requirements

- **Docker** (with socket mounted in dev container)
- **ARM64 cross-compile tools** (`aarch64-linux-gnu-gcc/g++`)
- **CMake** 3.15+
- **make**

All are included in the VS Code dev container (`.devcontainer/Dockerfile.devcontainer`).

## Troubleshooting

### "Libraries not found" on `build` build
Run `./platt.sh build-libs` first to build and extract libraries.

### Build takes too long
- First `build-libs` build is slow (20-30 min) — this is normal
- Docker caches layers, so repeat `build-libs` is fast
- Use `build` for day-to-day development (~5-10 sec)

### Binary won't run on Pi: "Invalid ELF header"
Verify the binary is ARM64:
```bash
file build/rpi5-local/peripheralControl
# Should show: ELF 64-bit LSB executable, ARM aarch64
```

### IntelliSense can't find includes
- Verify libraries are extracted: `ls build/rpi5-libs-extracted/rpi/include`
- Reload VS Code window: **Ctrl+Shift+P** → **"Developer: Reload Window"**
- Rescan solutions: **Ctrl+Shift+P** → **"C/C++: Rescan Solutions"**

### Out of disk space
Clean up Docker:
```bash
docker system prune -a
```

## Optimization Flags

The build applies CPU-specific optimizations for Raspberry Pi 5:

```
-march=armv8-a           # ARMv8-A instruction set
-mtune=cortex-a76        # Tune for Pi 5's Cortex-A76 cores
-O3                      # Maximum optimization level
```

These ensure fast execution on real hardware.

## CI/CD Integration

Example GitHub Actions workflow:

```yaml
- name: Build RPi5 Binary
  run: |
    ./platt.sh build-libs
    ./platt.sh build

- name: Upload Binary
  uses: actions/upload-artifact@v3
  with:
    name: peripheralControl-rpi5
    path: build/rpi5-local/peripheralControl
```

For shared library caching, push the Docker image to a registry:

```bash
docker tag platt2-rpi5-libs:latest myregistry.com/platt2-rpi5-libs:latest
docker push myregistry.com/platt2-rpi5-libs:latest
```

Then in CI, pull before building:
```bash
docker pull myregistry.com/platt2-rpi5-libs:latest
docker tag myregistry.com/platt2-rpi5-libs:latest platt2-rpi5-libs:latest
```
