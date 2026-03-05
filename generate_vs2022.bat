@echo off
setlocal

echo === fbx2md5 - Generate Visual Studio 2022 Solution ===
echo.

:: Clean old build if it exists (avoids stale CMake cache issues)
if exist "build" (
    echo Cleaning old build directory...
    rmdir /s /q build
)

mkdir build
cd build

:: Configure - generates .sln and .vcxproj files
:: FBX SDK is auto-detected from common install locations.
:: Override with: cmake -DFBX_SDK_ROOT="C:\path\to\sdk" ..
echo Configuring...
cmake -G "Visual Studio 17 2022" -A x64 -DFBX_STATIC=ON ..
if %errorlevel% neq 0 (
    echo.
    echo ERROR: CMake configure failed.
    echo.
    echo If FBX SDK was not found, install it or pass the path:
    echo   cmake -G "Visual Studio 17 2022" -A x64 -DFBX_SDK_ROOT="C:\Program Files\Autodesk\FBX\FBX SDK\2020.3.7" ..
    pause
    exit /b 1
)

echo.
echo === Solution generated successfully ===
echo.
echo Solution file: %cd%\fbx2md5.sln
echo.
echo Opening in Visual Studio...
start fbx2md5.sln

pause
