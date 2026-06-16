#!/bin/sh
# Flash the loading-stand firmware via the Pi's ST-Link using openocd.
# Invoked by flash_wifi.bat over SSH:   PIPW=<sudo-password> sh pi_flash.sh
# (flash_wifi.bat strips CR line-endings before running this.)
set -e

ELF="${ELF:-/home/pi/LoadingStandBOR0394000.elf}"

echo "[pi] stopping any running openocd (frees the ST-Link)..."
echo "$PIPW" | sudo -S pkill -x openocd 2>/dev/null || true
sleep 1

echo "[pi] programming $ELF ..."
echo "$PIPW" | sudo -S openocd -f interface/stlink.cfg -f target/stm32g0x.cfg \
    -c "program $ELF verify reset exit"

echo "[pi] done."
