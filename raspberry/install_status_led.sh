#!/usr/bin/env bash
# Instal·la el servei status-led: en arrencar, la matriu de LEDs de la
# Sense HAT es posa verda quan la Raspberry està preparada per fer un launch
# (vegeu src/codi_principal/launcher/status_led.py).
#
# Ús (des de l'arrel del repositori, a la Raspberry, amb el workspace compilat):
#   sudo ./raspberry/install_status_led.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Executa-ho amb sudo" >&2
    exit 1
fi
USER_NAME="${SUDO_USER:-}"
if [ -z "$USER_NAME" ] || [ "$USER_NAME" = root ]; then
    echo "Executa-ho amb sudo des del teu usuari (no com a root)" >&2
    exit 1
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ ! -f "$REPO_DIR/install/setup.bash" ]; then
    echo "Falta $REPO_DIR/install: compila primer amb colcon build" >&2
    exit 1
fi

# La matriu de LEDs és un framebuffer (/dev/fb*), del grup video.
usermod -aG video "$USER_NAME"

sed -e "s|@USER@|$USER_NAME|g" -e "s|@REPO_DIR@|$REPO_DIR|g" \
    "$REPO_DIR/raspberry/status-led.service" > /etc/systemd/system/status-led.service
systemctl daemon-reload
systemctl enable status-led.service
systemctl restart status-led.service
systemctl --no-pager status status-led.service || true

echo "Fet. Els LEDs s'haurien de posar verds en uns segons."
