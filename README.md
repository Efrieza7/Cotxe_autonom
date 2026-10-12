# Cotxe autonom
Projecte TDR: disseny i implementació d'un prototip de cotxe autònom.
Integro sensors (IMU, LIDAR) i ROS 2 per la comunicació entre nodes.
Aquesta versió inclou la integració del driver LDRobot LDLiDAR dins del mateix workspace.
Desenvolupo i entreno models de percepció i un algorisme de control per a la navegació.
Provo el sistema en simulació i en pista reduïda per validar detecció, estimació d'estat i maniobres segures.

## Comado per inicialitzar ros2
```bash
source /opt/ros/jazzy/setup.bash && 
source install/setup.bash 

# crear venv nou
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
# instal·lar les deps del planner
pip install -r src/codi_principal/nodes/path_planning/ft-fsd-path-planning-main/requirements.txt
# instal·lar altres deps del projecte (si tens un requirements global)
# pip install -r requirements.txt

```

## Raspberry Pi: GPIO segurs i prova de motors

A Ubuntu 24.04 (ROS 2 Jazzy) el paquet `RPi.GPIO` original no pot llegir
l'encoder (`Failed to add edge detection`). Cal el substitut compatible:

```bash
sudo apt remove python3-rpi.gpio
sudo apt install python3-rpi-lgpio
```

Perquè en engegar o reiniciar la Raspberry cap motor es mogui, i perquè els
nodes puguin fer servir els GPIO sense `sudo` (una sola vegada):

```bash
sudo ./raspberry/install_gpio_reset.sh   # config.txt + permisos gpio + servei gpio-reset
sudo reboot
```

Si surt `can not open gpiochip`, l'usuari no té permís per obrir
`/dev/gpiochip*`: torna a executar l'script i comprova que `groups` inclogui `gpio`.

Si un motor es queda engegat: `sudo python3 src/codi_principal/launcher/gpio_reset.py`.

Prova tipus missió d'inspecció (cotxe **aixecat**, menys de 10 s): els motors
giren, la direcció fa un sinus, frena i s'atura.

```bash
colcon build --packages-select my_pakage && source install/setup.bash
ros2 launch my_pakage test_motors.launch.py          # duty:=40.0 per anar més ràpid
```

## Raspberry Pi sense monitor (SSH) i LEDs d'estat

Per fer-la servir sense monitor ni teclat, des d'un altre ordinador (una sola vegada,
a la Raspberry):

```bash
sudo apt install openssh-server
sudo systemctl enable --now ssh
sudo systemctl set-default multi-user.target   # opcional: no engega l'escriptori (més RAM i CPU)
                                               # per tornar-lo a tenir: graphical.target
```

Després, des de l'ordinador (a la mateixa xarxa): `ssh frieza@frieza-desktop.local`
(o amb la IP, que surt amb `hostname -I`).

La matriu de LEDs de la Sense HAT mostra l'estat del cotxe
(`src/codi_principal/launcher/status_led.py`):

| LEDs | Estat |
|---|---|
| apagats | la Raspberry encara arrenca |
| verd fix | preparada per fer qualsevol launch |
| groc fix | s'executa un launch, el cotxe està aturat |
| vermell parpellejant | el cotxe es mou (`speed_control` o `test_motors` publiquen `car_moving`) |

S'instal·la una sola vegada, amb el workspace ja compilat:

```bash
sudo ./raspberry/install_status_led.sh
journalctl -u status-led -f     # per veure què fa
```

## Path Planning — bridge amb `ft-fsd-path-planning`

El node de path planning fa de pont entre els tòpics reals del projecte i
`ft-fsd-path-planning-main`, assumint cons **sense color**.

### Com funciona

1. `lidar_processing` publica els cons a `/ldlidar_node/cons_map` (`my_pakage_msgs/ConsMap`)
   amb format `[x, y, count, ...]`.
2. `path_planning` llegeix també la pose del vehicle a `/pose` (`Float32MultiArray`,
   format mínim `[x, y, yaw, ...]`).
3. El bridge separa els cons detectats en costat esquerre/dret segons la pose
   i el heading actual del vehicle, i executa:
   `PathPlanner.calculate_path_in_global_frame(...)`.
4. El resultat es publica a `/path_planning/waypoints` (`Float32MultiArray`,
   `[x0, y0, x1, y1, ...]`).

### Iniciar el node

```bash
ros2 run my_pakage path_planning
```

### Paràmetres configurables

| Paràmetre | Tipus | Valor per defecte | Descripció |
|---|---|---|---|
| `map_topic` | string | `/ldlidar_node/cons_map` | Tòpic de cons `ConsMap` |
| `pose_topic` | string | `/pose` | Tòpic de pose `[x,y,yaw,...]` |
| `path_topic` | string | `/path_planning/waypoints` | Tòpic de sortida de waypoints |
| `mission` | string | `trackdrive` | Missió del `PathPlanner` |
| `experimental_performance_improvements` | bool | `false` | Flag intern del planner |
| `min_cone_count` | int | `1` | Mínim `count` per con útil |
| `timer_period_sec` | float | `0.1` | Període de planificació |

```bash
ros2 run my_pakage path_planning \
  --ros-args -p mission:=autocross -p min_cone_count:=2
```

### Executar els tests

```bash
cd src/codi_principal
python -m pytest test/test_path_planning.py -v
```

### Fitxers del mòdul

```
src/codi_principal/nodes/path_planning/
    path_planner_bridge_node.py   # node ROS 2 bridge amb fsd_path_planning
    bridge_utils.py               # conversió de ConsMap i sortida del planner
    ft-fsd-path-planning-main/    # llibreria de path planning integrada
src/codi_principal/test/
    test_path_planning.py   # tests: recta, slalom, horquilla, degenerat
```
