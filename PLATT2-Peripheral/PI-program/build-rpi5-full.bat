@echo off
REM Two-step build: pre-built libs + app compilation
REM Usage: build-rpi5-full.bat [output-directory]

setlocal enabledelayedexpansion

set LIBS_IMAGE=rpi5-libs:latest
set APP_IMAGE=peripheralcontrol:build
set OUTPUT_DIR=.\peripheralControl_rpi5

if not "%~1"=="" (
    set OUTPUT_DIR=%~1
)

echo.
echo ======================================================
echo Raspberry Pi 5 Cross-Compilation Build System
echo ======================================================
echo.
echo Libs Image:  %LIBS_IMAGE%
echo App Image:   %APP_IMAGE%
echo Output:      %OUTPUT_DIR%
echo.

REM Step 1: Build pre-built libraries
echo [STEP 1] Building pre-built libraries image...
echo.
docker build -f Dockerfile.rpi5-libs -t %LIBS_IMAGE% .
if errorlevel 1 (
    echo ✗ Build failed: rpi5-libs
    pause
    exit /b 1
)
echo ✓ Libraries image built successfully
echo.

REM Step 2: Build app using the libraries
echo [STEP 2] Building application image...
echo.
docker build -f Dockerfile -t %APP_IMAGE% .
if errorlevel 1 (
    echo ✗ Build failed: app
    pause
    exit /b 1
)
echo ✓ Application image built successfully
echo.

REM Step 3: Extract binary
echo [STEP 3] Extracting binary...
echo.
if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"

docker create --name rpi5-extract %APP_IMAGE% /bin/sh >nul 2>&1
if errorlevel 1 (
    echo ✗ Container creation failed
    pause
    exit /b 1
)

docker cp rpi5-extract:/peripheralControl "%OUTPUT_DIR%\peripheralControl" >nul 2>&1
if errorlevel 1 (
    echo ✗ Extraction failed
    docker rm rpi5-extract >nul 2>&1
    pause
    exit /b 1
)

docker rm rpi5-extract >nul 2>&1

echo ✓ Binary extracted successfully
echo.

if exist "%OUTPUT_DIR%\peripheralControl" (
    echo ======================================================
    echo BUILD SUCCESSFUL!
    echo ======================================================
    echo.
    echo Binary Details:
    for /f "tokens=*" %%A in ('dir "%OUTPUT_DIR%\peripheralControl" ^| findstr peripheralControl') do echo   %%A
    echo.

) else (
    echo ✗ Build failed - binary not found
    pause
    exit /b 1
)

pause
endlocal
