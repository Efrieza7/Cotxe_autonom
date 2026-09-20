# car_simulator

Simulador per RViz per provar el `codi_principal` sense cotxe real:

- **Model del cotxe**: caixa (URDF) de 0.40 m (llarg / wheelbase) x 0.25 m
  (ample / track), publicat via `robot_state_publisher`.
- **Mapa de referència**: llegeix un fitxer YAML amb la posició `x, y` de
  cada con i el publica com a `MarkerArray` a `/simulator/ground_truth_map`.
- **LiDAR simulat**: publica `sensor_msgs/LaserScan` a `/ldlidar_node/scan`
  amb els paràmetres d'un LiDAR real (4500 punts/s, 10 voltes/s → 450
  punts/volta), només "veu" els cons a `max_range` (per defecte 2 m) del
  cotxe, i simula el desplaçament del cotxe punt a punt durant la volta
  (vegeu més avall). Substitueix el driver real, així que la resta del
  pipeline de mapeig (`lidar_image_creator`, `lidar_processing`,
  `cons_map_viz`) s'executa sense cap canvi, ja que és qui fa la correcció
  de posició.
- **Path**: visualitza `/path_planning/waypoints` com a `nav_msgs/Path` i
  `Marker` (línia verda) a `/simulator/path` / `/simulator/path_markers`.
- **Control**: el simulador NO decideix ni direcció ni velocitat. Només
  integra cinemàticament `/target_angle` i `/target_speed`, que ara publica
  `path_follower` (pure pursuit, dins `codi_principal`) a partir del path
  planificat i la pose. Com menys lògica de conducció hi hagi al simulador,
  més realista és la sortida.

## Canviar de mapa

Els mapes són fitxers YAML amb aquest format:

```yaml
cones:
  - {x: 1.0, y: 0.5}
  - {x: 1.0, y: -0.5}
```

Per generar-ne un de nou (pista ovalada d'exemple):

```bash
ros2 run car_simulator generate_map --output my_track.yaml \
  --length 10 --width 6 --track-width 3 --spacing 2
```

Per canviar de pista sense reiniciar, edita/substitueix el fitxer indicat a
`map_file`: els nodes `lidar_simulator_node` i `cone_map_publisher_node` el
recarreguen automàticament si detecten un canvi (paràmetre
`map_reload_period_sec`, per defecte cada 2 s).

## Executar

Cal llençar 3 launchers, per aquest ordre, en terminals separats:

```bash
# 1) Simulador: model del cotxe, mapa real, LiDAR simulat, path i RViz
ros2 launch car_simulator car_simulator.launch.py

# 2) Codi principal (my_pakage): mapeig, SENSE bycicle_mode ni nodes de hardware
ros2 launch my_pakage simulation_mapping.launch.py

# 3) Codi principal (my_pakage): planificació de trajectòria + seguiment (pure pursuit)
ros2 launch my_pakage path_planner_bridge.launch.py
```

El launcher (3) arrenca tant `path_planner_bridge` com `path_follower`: aquest
últim és qui calcula `/target_angle` i `/target_speed` a partir del path i la
pose, i és qui fa moure realment el cotxe simulat. Sense aquest launcher el
cotxe es queda quiet (`speed_mps` per defecte és `0.0`).

Paràmetres opcionals del simulador:

```bash
ros2 launch car_simulator car_simulator.launch.py map_file:=/ruta/al/meu_mapa.yaml max_range:=2.0
```

**No llencis** `proximiti_control.launch.py` ni `ldlidar_integration.launch.py`
en simulació: arrenquen `bycicle_mode` i nodes de hardware real (servo, motor,
driver del LiDAR físic, `proximiti_direccion`). `bycicle_mode` publicaria al
mateix `/pose` i `/bicycle_mode/pose` que ja genera `car_simulator_node`
(ground truth), i `proximiti_direccion` publicaria un `/target_angle` en
conflicte amb el de `path_follower`.

## Per què cal el desplaçament dels punts

El LiDAR simulat no calcula tots els punts d'una volta des d'una única pose:
cada punt es genera des de la pose instantània del cotxe en el moment exacte
en què s'hauria capturat (`lidar_simulator_node._pose_at_offset`, igual que
`lidar_image_creator.pose_at_offset` del codi principal). A 1 m/s, per
exemple, el primer punt d'una volta (0.1 s abans que l'últim) ja està
desplaçat uns quants cm respecte la posició final. Com que `lidar_image_creator`
sempre corregeix aquest desplaçament assumint que existeix, si el simulador no
el reproduís a l'`scan` en brut, la correcció introduiria un error en lloc
d'eliminar-lo.

