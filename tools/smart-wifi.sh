#!/bin/bash
# =============================================================================
# REFERENCE COPY (written by another developer) of the Raspberry Pi CM4
# access-point fallback script that lives on the Pi at:
#     /usr/local/bin/smart-wifi.sh
# launched by the systemd unit  smart-wifi.service  (oneshot, RemainAfterExit).
#
# It tries to join a known Wi-Fi network on boot; if none is found it raises a
# local hotspot named CM4-Config-<CRC(serial)> (password MySuperPassword123).
#
# Kept here so the loading-stand deployment is self-documented. The UART->TCP
# bridge (uart-bridge.service) is ordered After=smart-wifi.service so it comes
# up only once this script has settled the network.
# =============================================================================

# Ждем 20 секунд после старта системы, чтобы NetworkManager проснулся и
# успел подключиться к известной сети автоматически
sleep 20

# Проверяем статус подключения
CONNECTION_STATE=$(nmcli -t -f STATE general)

if [ "$CONNECTION_STATE" = "connected" ]; then
    echo "Wi-Fi подключен к домашней сети."
    exit 0
else
    echo "Домашняя сеть не найдена, поднимаем точку доступа."

    # Генерация имени (оставляем твою уникальную логику)
    SERIAL=$(grep Serial /proc/cpuinfo | awk '{print $3}')
    CRC_VAL=$(echo -n "$SERIAL" | cksum | awk '{print $1}')
    SSID_SUFFIX=$(printf "%X" $CRC_VAL)
    AP_SSID="CM4-Config-$SSID_SUFFIX"
    AP_PASSWORD="MySuperPassword123"

    # Поднимаем точку доступа
    nmcli connection delete hotspot 2>/dev/null
    nmcli device wifi hotspot ifname wlan0 ssid "$AP_SSID" password "$AP_PASSWORD" con-name hotspot
fi
