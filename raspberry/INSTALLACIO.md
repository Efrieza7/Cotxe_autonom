# Instal·lació de la Raspberry Pi del cotxe

Estat de la SD tal com estava el 23/10/2026, llegit directament de la targeta,
abans de reinstal·lar-la (tenia el sistema de fitxers danyat per caigudes de
tensió). L'ordre de les seccions és l'ordre per tornar-la a deixar igual.

El codi del repositori que hi havia era idèntic al commit `57e0fd3` de GitHub
(sense fitxers propis de la Raspberry): no cal recuperar res d'allà.

El 24/10/2026 es va reinstal·lar de zero amb aquests passos sobre **Ubuntu
24.04.5 Server** (sense escriptori). Les seccions 2-5 són exactament el que es
va instal·lar.

## 1. Sistema

- Raspberry Pi 4 Model B, **Ubuntu 24.04.5 Server** (imatge
  `ubuntu-24.04.5-preinstalled-server-arm64+raspi.img.xz`, kernel
  `6.8.0-*-raspi`). Usuari `frieza`, hostname `frieza-desktop`.
- Primer arrencada sense pantalla: abans de posar la SD a la Raspberry, a la
  partició `system-boot` es canvien `user-data` (usuari, contrasenya, hostname,
  SSH, zona horària, `cfg80211.conf`) i `network-config` (les dues WiFi amb
  `regulatory-domain: ES`). Així arrenca ja connectada i amb SSH.
- Trobar-la a la xarxa pel nom (`ssh frieza@frieza-desktop.local`), encara que
  canviï la IP:
  ```bash
  sudo apt install avahi-daemon
  ```
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

Paquets de ROS i del cotxe:

```bash
sudo apt install ros-jazzy-ros-base ros-dev-tools \
    ros-jazzy-nav2-lifecycle-manager ros-jazzy-nav2-util ros-jazzy-nav2-msgs \
    ros-jazzy-diagnostic-updater ros-jazzy-bondcpp ros-jazzy-robot-state-publisher \
    ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common \
    cmake libudev-dev python3-pip python3.12-venv python3-serial python3-numpy python3-scipy
```

No cal `ros-jazzy-desktop` ni `rviz2` / `slam_toolbox` (dependències de
`ldlidar_node` i `car_simulator` només per visualitzar): RViz s'obre al PC.

## 3. GPIO

A Ubuntu 24.04 cal el substitut de `RPi.GPIO`:

```bash
sudo apt remove python3-rpi.gpio
sudo apt install python3-rpi-lgpio
```

IMU de la Sense HAT (`imu_reader` la llegeix directament per I2C, sense la
llibreria `sense_hat`, que a Ubuntu no detecta la IMU):

```bash
sudo apt install python3-smbus2 i2c-tools
sudo usermod -aG i2c frieza
i2cdetect -y 1      # 6a = giroscopi/acceleròmetre (LSM9DS1)
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
sudo ./raspberry/install_gpio_reset.sh    # config.txt (motors a 0 + PWM del servo al GPIO 18), regles udev de gpiochip i pwm, grup gpio, servei gpio-reset
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
groups                             # ha d'incloure gpio, video i i2c
systemctl status gpio-reset status-led --no-pager
grep -E "^(gpio=|dtoverlay=pwm)" /boot/firmware/config.txt   # gpio=5,6,16,26=op,dl,pd i dtoverlay=pwm,pin=18,func=2
ls /sys/class/pwm/                 # pwmchip0 (PWM de maquinari del servo)
vcgencmd get_throttled             # 0x0 = sense caigudes de tensió
```

## Resum del que hi havia (per comprovar)

| Què | Valor |
|---|---|
| `config.txt` (afegit) | `gpio=4,5,6,16,26=op,dl,pd` (la resta, el de fàbrica d'Ubuntu, amb I2C i SPI activats). Ara el servo és al GPIO 18 (pin 12): l'script posa `gpio=5,6,16,26=op,dl,pd` i `dtoverlay=pwm,pin=18,func=2` |
| Regles udev pròpies | `99-gpio.rules`, `99-ldlidar.rules` |
| Serveis propis | `gpio-reset.service` (sysinit), `status-led.service` (multi-user) |
| Grups de `frieza` | `sudo`, `video`, `gpio` (+ els de fàbrica) |
| `.bashrc` | sense canvis (ROS es carrega a mà amb `source`) |
| Arrencada | `multi-user.target` (sense escriptori) |
| Paquets pip del sistema | cap (tot va al `.venv`) |
| Cau de numba | `~/.cache/cotxe_autonom/numba` (es regenera sola) |
