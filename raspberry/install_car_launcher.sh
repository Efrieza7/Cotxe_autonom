#!/usr/bin/env bash
# Instal·la el servei car-launcher: en arrencar la Raspberry espera el botó
# del joystick de la Sense HAT (LEDs blaus) i engega full_car.launch.py
# (vegeu src/codi_principal/launcher/start_car.py).
#
# Ús (des de l'arrel del repositori, a la Raspberry, amb el workspace compilat):
#   sudo ./raspberry/install_car_launcher.sh
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

# El joystick és /dev/input/event* (grup input); la resta (GPIO, I2C, LiDAR
# per USB) ja ho necessitava full_car.launch.py.
usermod -aG input,gpio,i2c,dialout,video "$USER_NAME"

sed -e "s|@USER@|$USER_NAME|g" -e "s|@REPO_DIR@|$REPO_DIR|g" \
    "$REPO_DIR/raspberry/car-launcher.service" > /etc/systemd/system/car-launcher.service
systemctl daemon-reload
systemctl enable car-launcher.service
systemctl restart car-launcher.service
systemctl --no-pager status car-launcher.service || true

echo "Fet. Quan els LEDs es posin blaus, prem el botó del joystick per engegar."
echo "Per desactivar-ho: sudo systemctl disable --now car-launcher.service"
