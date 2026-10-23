# Instal·lació de la Raspberry Pi del cotxe

Estat de la SD tal com estava el 23/10/2026, llegit directament de la targeta,
abans de reinstal·lar-la (tenia el sistema de fitxers danyat per caigudes de
tensió). L'ordre de les seccions és l'ordre per tornar-la a deixar igual.

El codi del repositori que hi havia era idèntic al commit `57e0fd3` de GitHub
(sense fitxers propis de la Raspberry): no cal recuperar res d'allà.

## 1. Sistema

- Raspberry Pi 4 Model B, **Ubuntu 24.04 LTS** (imatge *Raspberry Pi Generic*,
  kernel `6.8.0-*-raspi`). Usuari `frieza`, hostname `frieza-desktop`.
- Sense escriptori en arrencar:
  ```bash
  sudo systemctl set-default multi-user.target
  ```
- SSH:
  ```bash
  sudo apt install openssh-server
  sudo systemctl enable --now ssh
  ```
- WiFi amb `nmcli` (xarxa de casa i el mòbil com a punt d'accés):
  ```bash
  nmcli dev wifi connect "<xarxa>" password "<contrasenya>"
  ```
- País del WiFi (sense això no trobava algunes xarxes):
  ```bash
  echo "options cfg80211 ieee80211_regdom=ES" | sudo tee /etc/modprobe.d/cfg80211.conf
  ```
- Locale UTF-8 (requisit de ROS 2):
  ```bash
  sudo apt update && sudo apt install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  ```
- Git:
  ```bash
  sudo apt install git -y
  git config --global user.name "Efrieza"
  git config --global user.email "senricbe@gmail.com"
  ```

## 2. ROS 2 Jazzy

Seguint la guia oficial (repositori `universe` + `ros2-apt-source`):

```bash
sudo apt install software-properties-common curl
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade
```

Paquets que hi havia instal·lats:

```bash
sudo apt install ros-jazzy-ros-base ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-util ros-jazzy-diagnostic-updater
sudo apt install colcon cmake libudev-dev python3-pip python3.12-venv \
    python3-serial python3-numpy python3-scipy
```

També hi havia `ros-jazzy-desktop` (RViz, etc.). Amb SSH i sense escriptori
no es fa servir: es pot deixar només `ros-base` (ocupa menys i va més lleuger).

## 3. GPIO

A Ubuntu 24.04 cal el substitut de `RPi.GPIO`:

```bash
sudo apt remove python3-rpi.gpio
sudo apt install python3-rpi-lgpio
```

## 4. Repositori i compilació

```bash
cd ~
git clone https://github.com/Efrieza7/Cotxe_autonom.git
cd Cotxe_autonom
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

L'entorn de Python del path planning (`.venv`): vegeu el README principal.

## 5. Configuració pròpia del cotxe

Un cop compilat el workspace, des de l'arrel del repositori:

```bash
sudo ./raspberry/install_gpio_reset.sh    # config.txt, regla udev de gpiochip, grup gpio, servei gpio-reset
sudo ./raspberry/install_status_led.sh    # servei dels LEDs de la Sense HAT, grup video
```

Regla del LiDAR (`/dev/ldlidar`):

```bash
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ldlidar"' | sudo tee /etc/udev/rules.d/99-ldlidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Reinicia i comprova:

```bash
ls -l /dev/ldlidar                 # -> ttyUSB0
groups                             # ha d'incloure gpio i video
systemctl status gpio-reset status-led --no-pager
grep "^gpio=" /boot/firmware/config.txt   # gpio=4,5,6,16,26=op,dl,pd
vcgencmd get_throttled             # 0x0 = sense caigudes de tensió
```

## Resum del que hi havia (per comprovar)

| Què | Valor |
|---|---|
| `config.txt` (afegit) | `gpio=4,5,6,16,26=op,dl,pd` (la resta, el de fàbrica d'Ubuntu, amb I2C i SPI activats) |
| Regles udev pròpies | `99-gpio.rules`, `99-ldlidar.rules` |
| Serveis propis | `gpio-reset.service` (sysinit), `status-led.service` (multi-user) |
| Grups de `frieza` | `sudo`, `video`, `gpio` (+ els de fàbrica) |
| `.bashrc` | sense canvis (ROS es carrega a mà amb `source`) |
| Arrencada | `multi-user.target` (sense escriptori) |
| Paquets pip del sistema | cap (tot va al `.venv`) |
| Cau de numba | `~/.cache/cotxe_autonom/numba` (es regenera sola) |
