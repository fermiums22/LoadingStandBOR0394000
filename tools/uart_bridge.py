#!/usr/bin/env python3
"""Transparent serial<->TCP bridge for the loading-stand Nucleo.

Runs on the Raspberry Pi that hosts the Nucleo. Forwards bytes both ways
between a TCP port and the UART the stand's console is wired to (USART1 ->
/dev/ttyAMA5 by default). Any TCP client connects to this port, reads the
`DATA,...` telemetry stream and sends console commands (set_m, stream on, ...).
See the "Remote control over Wi-Fi" section of the README for the protocol.

Pure Python 3 stdlib, no pyserial/socat needed. Baud is set with `stty`.

Usage:  python3 uart_bridge.py [device] [baud] [tcp_port]
Default: /dev/ttyAMA5 921600 5555
"""
import os
import select
import socket
import subprocess
import sys

DEV  = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA5"
BAUD = sys.argv[2] if len(sys.argv) > 2 else "921600"
PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 5555


def main() -> None:
    # Raw 8N1 at the requested baud; no echo, no CR/LF translation.
    subprocess.run(["stty", "-F", DEV, BAUD, "raw", "-echo", "-onlcr"], check=True)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT))
    srv.listen(1)
    print(f"bridge {DEV}@{BAUD} <-> tcp:0.0.0.0:{PORT}", flush=True)

    while True:
        conn, addr = srv.accept()
        print(f"client connected: {addr}", flush=True)
        fd = os.open(DEV, os.O_RDWR | os.O_NOCTTY)
        try:
            conn.setblocking(False)
            while True:
                r, _, _ = select.select([conn, fd], [], [], 1.0)
                if conn in r:
                    data = conn.recv(4096)
                    if not data:
                        break                      # client closed
                    os.write(fd, data)             # PC -> Nucleo (commands)
                if fd in r:
                    try:
                        data = os.read(fd, 4096)
                    except OSError:
                        data = b""
                    if data:
                        try:
                            conn.sendall(data)     # Nucleo -> PC (telemetry)
                        except OSError:
                            break
        except Exception as e:                     # noqa: BLE001
            print(f"error: {e}", flush=True)
        finally:
            os.close(fd)
            conn.close()
            print("client gone", flush=True)


if __name__ == "__main__":
    main()
