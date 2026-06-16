@echo off
setlocal enabledelayedexpansion
REM ===========================================================================
REM  Build the loading-stand firmware and flash it OVER WIFI via the Raspberry
REM  Pi that hosts the Nucleo's ST-Link. Steps:
REM    1) build Debug (.elf) with the STM32 GNU toolchain + mingw32-make
REM    2) scp the .elf and pi_flash.sh to the Pi
REM    3) run openocd on the Pi: program + verify + reset
REM ===========================================================================

REM ---- edit these for your setup --------------------------------------------
set "PI_HOST=192.168.0.104"
set "PI_USER=pi"
set "PI_PW=pi2026bork"
set "PI_HOSTKEY=SHA256:inUMaugyGl4MzujYwXgqDYWP3kj13RkH6RFdImd2zW4"
REM ---------------------------------------------------------------------------

set "ROOT=%~dp0"
set "DEBUG_DIR=%ROOT%Debug"
set "ELF=%DEBUG_DIR%\LoadingStandBOR0394000.elf"
set "PLINK=C:\Program Files\PuTTY\plink.exe"
set "PSCP=C:\Program Files\PuTTY\pscp.exe"
set "GCC_BIN=C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.100.202509120712\tools\bin"
set "MINGW=C:\msys64\mingw64\bin"

if not exist "%PLINK%" ( echo [ERROR] plink not found: "%PLINK%" & pause & exit /b 1 )
if not exist "%PSCP%"  ( echo [ERROR] pscp not found:  "%PSCP%"  & pause & exit /b 1 )
if not exist "%DEBUG_DIR%\makefile" ( echo [ERROR] Debug makefile missing. & pause & exit /b 1 )
if not exist "%GCC_BIN%\arm-none-eabi-gcc.exe" ( echo [ERROR] STM32 GNU tools not found: "%GCC_BIN%" & pause & exit /b 1 )

where mingw32-make >nul 2>nul
if errorlevel 1 set "PATH=%MINGW%;%PATH%"
set "PATH=%GCC_BIN%;%PATH%"

echo.
echo [1/4] Building Debug firmware...
pushd "%DEBUG_DIR%" >nul
mingw32-make all -j4
set "RC=%ERRORLEVEL%"
popd >nul
if not "%RC%"=="0" ( echo [ERROR] firmware build failed. & pause & exit /b %RC% )
if not exist "%ELF%" ( echo [ERROR] ELF not produced: "%ELF%" & pause & exit /b 1 )

echo.
echo [2/4] Copying ELF to %PI_USER%@%PI_HOST% ...
"%PSCP%" -scp -batch -hostkey %PI_HOSTKEY% -pw %PI_PW% "%ELF%" %PI_USER%@%PI_HOST%:/home/pi/LoadingStandBOR0394000.elf
if errorlevel 1 ( echo [ERROR] scp ELF failed. & pause & exit /b 1 )

echo.
echo [3/4] Copying flash helper ...
"%PSCP%" -scp -batch -hostkey %PI_HOSTKEY% -pw %PI_PW% "%ROOT%pi_flash.sh" %PI_USER%@%PI_HOST%:/home/pi/pi_flash.sh
if errorlevel 1 ( echo [ERROR] scp pi_flash.sh failed. & pause & exit /b 1 )

echo.
echo [4/4] Flashing via openocd on the Pi...
"%PLINK%" -ssh -batch -hostkey %PI_HOSTKEY% -pw %PI_PW% %PI_USER%@%PI_HOST% "sed -i 's/\r$//' /home/pi/pi_flash.sh; PIPW=%PI_PW% sh /home/pi/pi_flash.sh"
if errorlevel 1 ( echo [ERROR] flash failed. & pause & exit /b 1 )

echo.
echo [OK] Firmware built and flashed over wifi via %PI_HOST%.
echo      (The on-Pi openocd gdb server is stopped after programming; it
echo       restarts automatically on the next Pi boot.)
pause
endlocal
