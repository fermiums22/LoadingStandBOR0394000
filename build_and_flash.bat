@echo off
setlocal enabledelayedexpansion

REM ===========================================================================
REM  BORK Loading Stand - build Debug configuration and flash Nucleo via ST-LINK
REM ===========================================================================

set "ROOT=%~dp0"
set "DEBUG_DIR=%ROOT%Debug"
set "ELF=%DEBUG_DIR%\LoadingStandBOR0394000.elf"
set "PROG=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set "GCC_BIN=C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.100.202509120712\tools\bin"

if not exist "%DEBUG_DIR%\makefile" (
    echo [ERROR] Debug makefile not found: "%DEBUG_DIR%\makefile"
    pause
    exit /b 1
)

where mingw32-make >nul 2>nul
if errorlevel 1 (
    if exist "C:\msys64\mingw64\bin\mingw32-make.exe" (
        set "PATH=C:\msys64\mingw64\bin;%PATH%"
    ) else (
        echo [ERROR] mingw32-make not found. Install MSYS2/MinGW or add mingw32-make to PATH.
        pause
        exit /b 1
    )
)

if exist "%GCC_BIN%\arm-none-eabi-gcc.exe" (
    set "PATH=%GCC_BIN%;%PATH%"
) else (
    echo [ERROR] GNU Tools for STM32 not found:
    echo         "%GCC_BIN%"
    echo         Update GCC_BIN in this script if CubeIDE is installed elsewhere.
    pause
    exit /b 1
)

if not exist "%PROG%" (
    echo [ERROR] STM32CubeProgrammer CLI not found:
    echo         "%PROG%"
    pause
    exit /b 1
)

echo.
echo [1/2] Building Debug...
pushd "%DEBUG_DIR%" >nul
mingw32-make all -j4
set "BUILD_RC=%ERRORLEVEL%"
popd >nul
if not "%BUILD_RC%"=="0" (
    echo [ERROR] Build failed.
    pause
    exit /b %BUILD_RC%
)

if not exist "%ELF%" (
    echo [ERROR] ELF not found after build:
    echo         "%ELF%"
    pause
    exit /b 1
)

echo.
echo [2/2] Flashing Nucleo via ST-LINK...
"%PROG%" -c port=SWD mode=UR -w "%ELF%" -v -rst
set "FLASH_RC=%ERRORLEVEL%"
if not "%FLASH_RC%"=="0" (
    echo [ERROR] Flash failed.
    pause
    exit /b %FLASH_RC%
)

echo.
echo [OK] Build and flash complete.
pause
endlocal
