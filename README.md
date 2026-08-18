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

## Path Planning — algorisme colorblind d'autocross

Implementació d'un planificador de ruta sense necessitat de conèixer el color dels cons,
inspirat en [papalotis/ft-fsd-path-planning](https://github.com/papalotis/ft-fsd-path-planning).

### Com funciona

1. El node `lidar_processing` publica els cons detectats al tòpic `/ldlidar_node/cons_map`
   com a `ConsMap` (interleaved `[x, y, count, ...]`).
2. El node `path_planning` subscriu aquest tòpic, construeix una triangulació de Delaunay
   dels cons i extreu els punts mig de les arestes que creuen el límit de pista.
3. Els punts s'ordenen amb un heurístic de veí més proper i s'aplica un suavitzat
   per finestra mòbil.
4. El resultat es publica com a `Float32MultiArray` a `/path_planning/waypoints`
   (interleaved `[x0, y0, x1, y1, ...]`).

### Iniciar el node

```bash
ros2 run codi_principal path_planning
```

### Paràmetres configurables

| Paràmetre        | Tipus  | Valor per defecte | Descripció                                    |
|------------------|--------|-------------------|-----------------------------------------------|
| `smooth_window`  | int    | 3                 | Mida de la finestra del suavitzat             |
| `min_edge_length`| float  | 0.3               | Longitud mínima d'aresta Delaunay (m)         |
| `max_edge_length`| float  | 6.0               | Longitud màxima d'aresta Delaunay (m)         |
| `min_cone_count` | int    | 2                 | Observacions mínimes per acceptar un con      |

```bash
ros2 run codi_principal path_planning \
  --ros-args -p smooth_window:=5 -p max_edge_length:=4.0
```

### Executar els tests

```bash
cd src/codi_principal
python -m pytest test/test_path_planning.py -v
```

### Fitxers del mòdul

```
src/codi_principal/nodes/path_planning/
    __init__.py      # marcador de paquet
    planner.py       # algoritme pur (sense ROS): compute_centerline()
    ros_node.py      # node ROS 2: PathPlanningNode
src/codi_principal/test/
    test_path_planning.py   # tests: recta, slalom, horquilla, degenerat
```
