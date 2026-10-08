"""Genera el mapa conceptual del sistema (Imatge 13 del document).

Executar des de la carpeta `imatges/`:
    python3 imatge_13_mapa_conceptual.py

Les coordenades són en píxels d'un llenç de 2000 x 851 (origen a dalt a l'esquerra).
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle

OUT = 'imatge_13_mapa_conceptual.png'

W, H = 2000, 851
INK = '#333333'
COLORS = {
    'location': '#dff5dd',
    'mapping': '#dcdcf8',
    'control': '#fcdcdc',
    'path': '#fdf39c',
    'none': '#ffffff',
}
FONT = 11

# Estils de fletxa (vegeu la llegenda)
SOLID = '-'                      # activació
DASHED = (0, (5, 4))             # informació
DASHDOT = (0, (9, 4, 1.5, 4))    # informació i activació

fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(H, 0)
ax.set_aspect('equal')
ax.axis('off')

shapes = {}


def _text_width(label):
    return max(len(line) for line in label.split('\n')) * 8.6


def node(key, x, y, label, color):
    """Node de ROS 2: forma arrodonida."""
    w = max(105, _text_width(label) + 34)
    h = 52
    p = FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                       boxstyle=f'round,pad=0,rounding_size={h / 2}',
                       fc=COLORS[color], ec=INK, lw=1.2, zorder=3)
    ax.add_patch(p)
    ax.text(x, y, label, ha='center', va='center', fontsize=FONT, zorder=4)
    shapes[key] = p


def topic(key, x, y, label, color):
    """Topic o lectura d'un sensor: rombe."""
    lines = label.count('\n') + 1
    w = max(104, _text_width(label) + 56)
    h = 78 if lines == 1 else 86
    p = Polygon([(x, y - h / 2), (x + w / 2, y), (x, y + h / 2), (x - w / 2, y)],
                closed=True, fc=COLORS[color], ec=INK, lw=1.2, zorder=3)
    ax.add_patch(p)
    ax.text(x, y, label, ha='center', va='center', fontsize=FONT, zorder=4, linespacing=1.4)
    shapes[key] = p


def actuator(key, x, y, label):
    """Actuador: hexàgon."""
    w, h = 104, 80
    c = 18
    p = Polygon([(x - w / 2 + c, y - h / 2), (x + w / 2 - c, y - h / 2), (x + w / 2, y),
                 (x + w / 2 - c, y + h / 2), (x - w / 2 + c, y + h / 2), (x - w / 2, y)],
                closed=True, fc=COLORS['none'], ec=INK, lw=1.2, zorder=3)
    ax.add_patch(p)
    ax.text(x, y, label, ha='center', va='center', fontsize=FONT, zorder=4, linespacing=1.4)
    shapes[key] = p


def block(x, y, label, color):
    """Quadre de color de la llegenda."""
    w, h = 105, 78
    ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h, fc=COLORS[color], ec=INK, lw=1.2, zorder=3))
    ax.text(x, y, label, ha='center', va='center', fontsize=FONT, zorder=4)


def edge(a, b, style, rad=0.0):
    p = FancyArrowPatch(
        _center(a),
        _center(b),
        patchA=shapes[a], patchB=shapes[b],
        connectionstyle=f'arc3,rad={rad}',
        arrowstyle='-|>,head_length=7,head_width=3.5',
        color=INK, lw=1.2, linestyle=style, zorder=2,
        shrinkA=1, shrinkB=1,
    )
    ax.add_patch(p)


def _center(key):
    s = shapes[key]
    if isinstance(s, FancyBboxPatch):
        return (s.get_x() + s.get_width() / 2, s.get_y() + s.get_height() / 2)
    xy = s.get_xy()
    return (xy[:, 0].min() + xy[:, 0].max()) / 2, (xy[:, 1].min() + xy[:, 1].max()) / 2


def polyline(points, style, label=None, label_at=None):
    xs, ys = zip(*points)
    ax.plot(xs[:-1], ys[:-1], color=INK, lw=1.2, linestyle=style, zorder=2)
    ax.add_patch(FancyArrowPatch(points[-2], points[-1], arrowstyle='-|>,head_length=7,head_width=3.5',
                                 color=INK, lw=1.2, linestyle=style, zorder=2, shrinkA=0, shrinkB=0))
    if label:
        ax.text(*label_at, label, ha='center', va='center', fontsize=9, fontweight='bold',
                bbox=dict(fc='white', ec='none', pad=1.5), zorder=4)


# ---------------------------------------------------------------- llegenda
node('leg_node', 78, 99, 'Node', 'none')
topic('leg_topic', 565, 99, 'Topics', 'none')
actuator('leg_act', 789, 99, 'Actuadors')
polyline([(78, 73), (78, 34), (565, 34), (565, 60)], DASHED, 'Informació', (404, 34))
polyline([(131, 99), (514, 99)], DASHDOT, 'Informació i activació', (323, 99))
polyline([(78, 125), (78, 163), (565, 163), (565, 138)], SOLID, 'Activació', (420, 163))
edge('leg_topic', 'leg_act', SOLID)
block(934, 99, 'Location', 'location')
block(1092, 99, 'Mapping', 'mapping')
block(1250, 99, 'Control', 'control')
block(1407, 99, 'Path Plan', 'path')

# ---------------------------------------------------------------- elements
topic('location_solved_prev', 447, 231, 'location_solved\nt-1', 'mapping')
topic('steering_prev', 329, 322, 'steering_angle\nt-1', 'control')
topic('imu', 118, 441, 'IMU', 'location')
topic('encoder', 118, 586, 'Motor\nencoder', 'location')
topic('scan', 118, 785, 'LaserScan', 'mapping')
topic('pose', 513, 692, 'pose', 'location')
topic('cons_map', 844, 534, 'cons_map', 'mapping')
topic('location_solved', 1107, 256, 'location_solved', 'mapping')
topic('waypoints', 1192, 785, 'waypoints', 'path')
topic('steering_angle', 1750, 744, 'steering_angle', 'control')

node('imu_reader', 329, 441, 'imu_reader', 'location')
node('motor_reader', 329, 586, 'motor_reader', 'location')
node('bicycle_model', 500, 441, 'bicycle_model', 'location')
node('lidar_processing', 850, 376, 'lidar_processing', 'mapping')
node('cone_map_viz', 1107, 429, 'cone_map_viz', 'mapping')
node('lidar_image_creator', 723, 784, 'lidar_image_creator', 'mapping')
node('path_planner_bridge', 955, 666, 'path_planner_bridge', 'path')
node('fsd_path_planning', 1192, 666, 'fsd_path_planning', 'path')
node('path_follower', 1396, 666, 'path_follower', 'control')
node('speed_control', 1581, 586, 'speed_control', 'control')
node('steering', 1581, 744, 'steering', 'control')

actuator('motor_driver', 1921, 586, 'Motor\ndriver')
actuator('steering_motor', 1921, 744, 'Steering\nmotor')

# ---------------------------------------------------------------- connexions
# Localització
edge('imu', 'imu_reader', DASHED)
edge('encoder', 'motor_reader', DASHED)
edge('scan', 'imu_reader', SOLID, rad=-0.25)
edge('scan', 'motor_reader', SOLID, rad=-0.2)
edge('imu_reader', 'bicycle_model', DASHDOT)
edge('motor_reader', 'bicycle_model', DASHDOT, rad=0.3)
edge('steering_prev', 'bicycle_model', DASHED, rad=-0.3)
edge('location_solved_prev', 'bicycle_model', DASHED, rad=0.15)
edge('bicycle_model', 'pose', DASHDOT, rad=0.3)

# Mapeig
edge('scan', 'lidar_image_creator', DASHED)
edge('pose', 'lidar_image_creator', DASHDOT, rad=0.2)
edge('pose', 'lidar_processing', DASHED, rad=-0.3)
edge('lidar_image_creator', 'lidar_processing', DASHDOT, rad=-0.25)
edge('lidar_processing', 'location_solved', DASHED, rad=0.25)
edge('lidar_processing', 'cons_map', DASHDOT, rad=-0.35)
edge('cons_map', 'lidar_processing', DASHED, rad=-0.35)
edge('cons_map', 'cone_map_viz', DASHDOT, rad=-0.3)

# Path plan
edge('pose', 'path_planner_bridge', DASHED, rad=-0.05)
edge('cons_map', 'path_planner_bridge', DASHDOT, rad=0.3)
edge('path_planner_bridge', 'fsd_path_planning', DASHDOT, rad=-0.3)
edge('fsd_path_planning', 'path_planner_bridge', DASHDOT, rad=-0.3)
edge('path_planner_bridge', 'waypoints', DASHDOT, rad=0.35)

# Control
edge('waypoints', 'path_follower', DASHDOT, rad=-0.3)
edge('path_follower', 'speed_control', DASHDOT, rad=-0.3)
edge('path_follower', 'steering', DASHDOT, rad=0.3)
edge('speed_control', 'motor_driver', SOLID)
edge('steering', 'steering_angle', DASHED)
edge('steering_angle', 'steering_motor', SOLID)

fig.savefig(OUT, dpi=150, facecolor='white')
print('Guardat', OUT)
