#!/usr/bin/env bash
# Instal·la el reset dels GPIO a la Raspberry:
#   1. config.txt: el firmware posa els pins dels motors i del servo a 0
#      abans que arrenqui Linux (és el més ràpid que es pot fer).
#   2. gpio-reset.service: torna a posar tots els pins a 0 quan Linux arrenca
#      i quan s'apaga.
#
# Ús (des de l'arrel del repositori, a la Raspberry):
#   sudo ./raspberry/install_gpio_reset.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Executa-ho amb sudo" >&2
    exit 1
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Pins BCM de sortida (servo i DRV8833). Han de coincidir amb full_car.launch.py.
CONFIG_LINE="gpio=4,22-25=op,dl"

if ! python3 -c "import RPi.GPIO" 2>/dev/null; then
    echo "Falta RPi.GPIO. A Ubuntu 24.04 instal·la: sudo apt install python3-rpi-lgpio" >&2
    exit 1
fi

# 1. config.txt (Ubuntu i Pi OS Bookworm: /boot/firmware, Pi OS antic: /boot)
CONFIG_TXT=/boot/firmware/config.txt
[ -f "$CONFIG_TXT" ] || CONFIG_TXT=/boot/config.txt
if grep -qxF "$CONFIG_LINE" "$CONFIG_TXT"; then
    echo "$CONFIG_TXT ja té: $CONFIG_LINE"
else
    cp "$CONFIG_TXT" "$CONFIG_TXT.bak"
    printf '\n[all]\n# Cotxe autonom: motors i servo a 0 en arrencar\n%s\n' "$CONFIG_LINE" >> "$CONFIG_TXT"
    echo "Afegit a $CONFIG_TXT (còpia a $CONFIG_TXT.bak): $CONFIG_LINE"
fi

# 2. Permisos: a Ubuntu /dev/gpiochip* només és de root i els nodes donen
#    "can not open gpiochip". Es dona accés al grup gpio.
groupadd -f gpio
cat > /etc/udev/rules.d/99-gpio.rules <<'EOF'
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
EOF
udevadm control --reload-rules
udevadm trigger --subsystem-match=gpio
if [ -n "${SUDO_USER:-}" ] && [ "$SUDO_USER" != root ]; then
    usermod -aG gpio "$SUDO_USER"
    echo "Usuari $SUDO_USER afegit al grup gpio (cal tornar a iniciar sessió)"
fi
ls -l /dev/gpiochip*

# 3. Servei systemd
install -m 755 "$REPO_DIR/src/codi_principal/launcher/gpio_reset.py" /usr/local/bin/gpio_reset.py
install -m 644 "$REPO_DIR/raspberry/gpio-reset.service" /etc/systemd/system/gpio-reset.service
systemctl daemon-reload
systemctl enable gpio-reset.service
systemctl restart gpio-reset.service
systemctl --no-pager status gpio-reset.service || true

echo "Fet. Reinicia la Raspberry per aplicar el config.txt."
