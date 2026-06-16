@echo off
setlocal
REM ===========================================================================
REM  Build the DriveScope pc_tool (Dear ImGui / ImPlot / GLFW, MinGW + Ninja).
REM  First run configures and downloads glfw/imgui/implot via FetchContent
REM  (needs internet); subsequent runs are fast incremental builds.
REM ===========================================================================

set "ROOT=%~dp0"
set "MINGW=C:\msys64\mingw64\bin"
set "SRC=%ROOT%pc_tool"
set "BUILD=%ROOT%build\pc_tool"

if not exist "%MINGW%\cmake.exe" (
    echo [ERROR] cmake not found in "%MINGW%".
    echo         Install MSYS2/MinGW or edit MINGW in this script.
    pause
    exit /b 1
)
set "PATH=%MINGW%;%PATH%"

echo.
echo [1/3] Configure (CMake + Ninja)...
if not exist "%BUILD%\CMakeCache.txt" (
    cmake -S "%SRC%" -B "%BUILD%" -G Ninja -DCMAKE_BUILD_TYPE=Debug
    if errorlevel 1 ( echo [ERROR] configure failed. & pause & exit /b 1 )
) else (
    echo     cache exists, skipping configure.
)

echo.
echo [2/3] Build...
cmake --build "%BUILD%" --config Debug
if errorlevel 1 ( echo [ERROR] build failed. & pause & exit /b 1 )

echo.
echo [3/3] Done. exe: "%BUILD%\drivescope.exe"
echo               also copied to "%SRC%\dist\DriveScope.exe"

REM Launch the freshly built app (comment out the next line to build only).
start "" "%SRC%\dist\DriveScope.exe"

echo [OK] pc_tool build complete.
pause
endlocal
