@echo off
REM ----------------------------------------------------------------------
REM  Generate a Visual Studio 2022 solution for fbx2md5 under build\.
REM  After this script finishes, open build\fbx2md5.sln in VS and build
REM  the fbx2md5 target (Release x64 recommended).
REM
REM  Requires:
REM    * CMake 3.15 or newer on PATH
REM    * Visual Studio 2022 with the "Desktop development with C++" workload
REM ----------------------------------------------------------------------

setlocal

if not exist build mkdir build

cmake -S . -B build -G "Visual Studio 17 2022" -A x64
if errorlevel 1 (
    echo.
    echo [FAILED] CMake generation failed. See messages above.
    exit /b 1
)

echo.
echo [OK] Solution generated: build\fbx2md5.sln
echo      Open it in Visual Studio 2022 and build the fbx2md5 target.
echo      The executable will appear in build\Release\fbx2md5.exe.

endlocal
