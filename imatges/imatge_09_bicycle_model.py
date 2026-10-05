import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, FancyBboxPatch, Polygon
from matplotlib.transforms import Affine2D

OUT = 'imatge_09_bicycle_model.png'

# Geometria (gir a l'esquerra, delta > 0)
L = 4.0
theta = math.radians(25)
delta = math.radians(28)
R = L / math.tan(delta)

u = (math.cos(theta), math.sin(theta))        # direcció del cotxe
n = (-math.sin(theta), math.cos(theta))       # normal a l'esquerra
Pr = (6.6, 0.0)                               # eix posterior (x, y)
Pf = (Pr[0] + L * u[0], Pr[1] + L * u[1])     # eix davanter
O = (Pr[0] + R * n[0], Pr[1] + R * n[1])      # centre de gir
Rf = L / math.sin(delta)

INK = '#222222'
BODY = '#555555'
ACC = '#c0392b'
BLUE = '#1f5fa8'

fig, ax = plt.subplots(figsize=(12, 7.2), dpi=110)
ax.set_aspect('equal')
ax.axis('off')
fig.patch.set_facecolor('white')


def wheel(c, ang, length=1.2, width=0.34, color=BODY):
    p = FancyBboxPatch((-length / 2, -width / 2), length, width,
                       boxstyle='round,pad=0,rounding_size=0.12',
                       fc=color, ec=INK, lw=1.2, zorder=5)
    p.set_transform(Affine2D().rotate(ang).translate(*c) + ax.transData)
    ax.add_patch(p)


def pt(p, d, k):
    return (p[0] + d[0] * k, p[1] + d[1] * k)


def right_angle(corner, d1, d2, s=0.28):
    a = pt(corner, d1, s)
    b = pt(a, d2, s)
    c = pt(corner, d2, s)
    ax.plot([a[0], b[0], c[0]], [a[1], b[1], c[1]], color=INK, lw=1, zorder=4)


# Trajectòria de l'eix posterior (arc centrat a O)
ang_r = math.degrees(math.atan2(Pr[1] - O[1], Pr[0] - O[0]))
ax.add_patch(Arc(O, 2 * R, 2 * R, theta1=ang_r - 25, theta2=ang_r + 55,
                 color=BLUE, lw=1.6, ls='--', zorder=1))
ax.text(*pt(O, (math.cos(math.radians(ang_r + 40)), math.sin(math.radians(ang_r + 40))), R + 0.35),
        'trajectòria', color=BLUE, fontsize=11, ha='center', rotation=ang_r + 40 - 90)

# Radis
ax.plot([O[0], Pr[0]], [O[1], Pr[1]], color=INK, lw=1.4, zorder=3)
ax.plot([O[0], Pf[0]], [O[1], Pf[1]], color=INK, lw=1.0, ls=':', zorder=3)
mid = pt(O, n, -R / 2)
ax.text(mid[0] + 0.25, mid[1] - 0.05, 'R', fontsize=16, color=INK, ha='left', va='top')
right_angle(Pr, (-n[0], -n[1]), (u[0], u[1]))
uf = (math.cos(theta + delta), math.sin(theta + delta))
nf = (-uf[1], uf[0])
right_angle(Pf, nf, (-uf[0], -uf[1]))

# Angle delta al centre de gir
a0 = math.degrees(math.atan2(Pr[1] - O[1], Pr[0] - O[0]))
a1 = math.degrees(math.atan2(Pf[1] - O[1], Pf[0] - O[0]))
ax.add_patch(Arc(O, 1.6, 1.6, theta1=min(a0, a1), theta2=max(a0, a1), color=ACC, lw=1.4))
am = math.radians((a0 + a1) / 2)
ax.text(O[0] + 1.05 * math.cos(am), O[1] + 1.05 * math.sin(am), 'δ', color=ACC, fontsize=15,
        ha='center', va='center')

# Cos del cotxe i rodes
ax.plot([Pr[0], Pf[0]], [Pr[1], Pf[1]], color=BODY, lw=6, solid_capstyle='round', zorder=4)
wheel(Pr, theta)
wheel(Pf, theta + delta)
ax.plot(*Pr, 'o', color=INK, ms=6, zorder=6)
ax.plot(*O, 'o', color=INK, ms=7, zorder=6)
ax.text(O[0] - 0.2, O[1] + 0.15, 'O', fontsize=15, ha='right', va='bottom')
ax.text(Pr[0] - 0.75, Pr[1] - 0.1, '(x, y)', fontsize=14, ha='right', va='center')

# Angle delta a la roda davantera (eix del cotxe vs roda)
ext = pt(Pf, u, 1.6)
ax.plot([Pf[0], ext[0]], [Pf[1], ext[1]], color=INK, lw=1, ls='--', zorder=3)
wdir = pt(Pf, uf, 1.6)
ax.plot([Pf[0], wdir[0]], [Pf[1], wdir[1]], color=INK, lw=1, zorder=3)
ax.add_patch(Arc(Pf, 2.2, 2.2, theta1=math.degrees(theta), theta2=math.degrees(theta + delta),
                 color=ACC, lw=1.4))
am = theta + delta / 2
ax.text(Pf[0] + 1.4 * math.cos(am), Pf[1] + 1.4 * math.sin(am), 'δ', color=ACC, fontsize=15,
        ha='center', va='center')

# Velocitat
v0 = pt(Pr, u, 0.7)
v1 = pt(Pr, u, 1.6)
off = pt((0, 0), n, 0.5)
ax.annotate('', xy=(v1[0] + off[0], v1[1] + off[1]), xytext=(v0[0] + off[0], v0[1] + off[1]),
            arrowprops=dict(arrowstyle='-|>', color=BLUE, lw=2))
vm = pt(pt(Pr, u, 1.15), n, 0.9)
ax.text(*vm, 'v', color=BLUE, fontsize=15, ha='center', va='center')

# Cota L
d = 1.3
a = pt(Pr, n, -d)
b = pt(Pf, n, -d)
ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='<->', color=INK, lw=1.2))
for p in (Pr, Pf):
    q1 = pt(p, n, -0.35)
    q2 = pt(p, n, -d - 0.15)
    ax.plot([q1[0], q2[0]], [q1[1], q2[1]], color=INK, lw=0.8)
lm = pt(pt(Pr, u, L / 2), n, -d - 0.35)
ax.text(*lm, 'L', fontsize=15, ha='center', va='center')

# Angle theta respecte de l'eix X
hx = (Pr[0] - 1.6, Pr[1])
ax.plot([Pr[0], Pr[0] + 2.6], [Pr[1], Pr[1]], color=INK, lw=0.9, ls='--', zorder=2)
ax.add_patch(Arc(Pr, 4.4, 4.4, theta1=0, theta2=math.degrees(theta), color=INK, lw=1.2))
ax.text(Pr[0] + 2.55 * math.cos(theta / 2), Pr[1] + 2.55 * math.sin(theta / 2), 'θ',
        fontsize=15, ha='center', va='center')

# Eixos de referència
ox, oy = 0.5, -1.9
ax.annotate('', xy=(ox + 2.0, oy), xytext=(ox, oy), arrowprops=dict(arrowstyle='-|>', color=INK, lw=1.4))
ax.annotate('', xy=(ox, oy + 2.0), xytext=(ox, oy), arrowprops=dict(arrowstyle='-|>', color=INK, lw=1.4))
ax.text(ox + 2.15, oy, 'X', fontsize=14, va='center')
ax.text(ox, oy + 2.2, 'Y', fontsize=14, ha='center')
ax.text(ox - 0.15, oy - 0.15, '0', fontsize=13, ha='right', va='top')

# Llegenda
legend = ['L - Distància entre eixos', 'v - Velocitat', 'δ - Angle de les rodes',
          'R - Radi de gir (eix posterior)', 'θ - Orientació', '(x, y) - Posició (eix posterior)',
          'O - Centre de gir']
for i, t in enumerate(legend):
    ax.text(11.4, 8.0 - i * 0.5, t, fontsize=13, va='top')

# Fórmules
fx, fy = 12.4, 0.2
ax.text(fx, fy + 1.3, r'$R = \dfrac{L}{\tan\delta}$', fontsize=17, ha='left')
ax.text(fx, fy, r'$\omega = \dfrac{v}{R} = \dfrac{v}{L}\,\tan\delta$', fontsize=17, ha='left')

ax.set_title('Model de Bicicleta per Localització de Vehicle', fontsize=20, pad=12)
ax.set_xlim(-0.2, 17.0)
ax.set_ylim(-2.6, 8.4)
fig.tight_layout()
fig.savefig(OUT, facecolor='white')
print(OUT, 'R=%.2f O=(%.2f, %.2f) Pf=(%.2f, %.2f)' % (R, *O, *Pf))
