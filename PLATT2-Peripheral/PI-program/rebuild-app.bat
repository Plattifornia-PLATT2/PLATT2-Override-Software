@echo off
REM Quick rebuild app only (assumes rpi5-libs:latest already built)
REM Usage: rebuild-app.bat [output-directory]

setlocal enabledelayedexpansion

set OUTPUT_DIR=.\peripheralControl_rpi5

if not "%~1"=="" (
    set OUTPUT_DIR=%~1
)

echo.
echo Rebuilding application only (fast, ~5 sec)...
echo Output: %OUTPUT_DIR%
echo.

REM Build app
docker build -f Dockerfile -t peripheralcontrol:build .
if errorlevel 1 (
    echo ✗ Build failed
    exit /b 1
)

REM Extract binary
if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"

docker create --name rpi5-extract peripheralcontrol:build /bin/sh >nul 2>&1
if errorlevel 1 (
    echo ✗ Container creation failed
    exit /b 1
)

docker cp rpi5-extract:/peripheralControl "%OUTPUT_DIR%\peripheralControl" >nul 2>&1
if errorlevel 1 (
    echo ✗ Extraction failed
    docker rm rpi5-extract >nul 2>&1
    exit /b 1
)

docker rm rpi5-extract >nul 2>&1

if exist "%OUTPUT_DIR%\peripheralControl" (
    echo.
    echo ======================================================
    echo ✓ REBUILD SUCCESSFUL!
    echo ======================================================
    echo.
    echo Binary: %OUTPUT_DIR%\peripheralControl
    echo.
    for /f "tokens=*" %%A in ('dir "%OUTPUT_DIR%\peripheralControl" ^| findstr peripheralControl') do echo File size: %%A
    echo.
) else (
    echo ✗ Rebuild failed
    exit /b 1
)

endlocal
