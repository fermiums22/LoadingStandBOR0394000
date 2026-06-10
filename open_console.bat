@echo off
setlocal enabledelayedexpansion

REM ===========================================================================
REM  BORK Loading Stand - open serial console (ST-Link Virtual COM Port)
REM ===========================================================================
REM  COM_PORT : used ONLY when several ST-Links are connected at once.
REM             If exactly one ST-Link is present, that one is used automatically.
REM ===========================================================================
set "COM_PORT=COM14"
set "BAUD=921600"

set "PUTTY=C:\Program Files\PuTTY\putty.exe"
set "PROG=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

REM --- Enumerate ST-Link Virtual COM Ports --------------------------------
set "COUNT=0"
for /f "delims=" %%P in ('powershell -NoProfile -Command "Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match 'STLink Virtual COM Port' } | ForEach-Object { if ($_.Name -match '\((COM\d+)\)') { $matches[1] } }"') do (
    set /a COUNT+=1
    set "PORT_!COUNT!=%%P"
)

if "%COUNT%"=="0" (
    echo [ERROR] No ST-Link Virtual COM Port found. Check the USB cable / driver.
    pause
    exit /b 1
)

if "%COUNT%"=="1" (
    set "USE=!PORT_1!"
    echo One ST-Link detected -^> using !USE!
) else (
    set "USE=%COM_PORT%"
    echo %COUNT% ST-Links detected -^> using configured %COM_PORT%
)

echo Opening !USE! @ %BAUD% 8N1 ...
start "" "%PUTTY%" -serial !USE! -sercfg %BAUD%,8,n,1,N

REM --- If a single ST-Link, reset the target so the BORK logo is shown -----
if "%COUNT%"=="1" (
    if exist "%PROG%" (
        timeout /t 1 /nobreak >nul
        echo Resetting target so the BORK logo appears...
        "%PROG%" -c port=SWD mode=HOTPLUG -rst >nul 2>&1
    )
)

echo Done. Type 'help' in the console window.
endlocal
