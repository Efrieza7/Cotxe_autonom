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

```

## Path Planning — bridge amb `ft-fsd-path-planning`

El node de path planning fa de pont entre els tòpics reals del projecte i
`ft-fsd-path-planning-main`, assumint cons **sense color**.

### Com funciona

1. `lidar_processing` publica els cons a `/ldlidar_node/cons_map` (`my_pakage_msgs/ConsMap`)
   amb format `[x, y, count, ...]`.
2. `path_planning` llegeix també la pose del vehicle a `/pose` (`Float32MultiArray`,
   format mínim `[x, y, yaw, ...]`).
3. El bridge converteix tots els cons a `ConeTypes.UNKNOWN` i executa:
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
