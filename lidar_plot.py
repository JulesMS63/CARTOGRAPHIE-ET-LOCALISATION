"""Dessins matplotlib partagés par l'application et l'export d'image.

Le même code dessine la vue « Carte » à l'écran et le fichier carte.png : ce qu'on voit est ce qu'on exporte.
"""

import logging
import warnings

import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle

from geometry import transform_points

# L'avertissement « Ignoring fixed x limits... » de matplotlib est normal ici : la vue est fixée par
# l'utilisateur (zoom, déplacement) et matplotlib ajuste l'autre axe pour garder une échelle 1:1.
warnings.filterwarnings("ignore", message="Ignoring fixed (x|y) limits")
logging.getLogger("matplotlib.axes._base").setLevel(logging.ERROR)     # même message, côté « logging »

# Jetons de couleur (les mêmes que la page web, thème clair)
CANVAS = "#fbfcfd"
GRID = "#e8ecf0"
AXIS = "#c2cad3"
INK = "#14202b"
MUTED = "#5b6875"
LIVE = "#1f3fa6"


def nice_step(span):
    """Pas des anneaux / de la grille : une valeur « ronde » donnant 4 à 8 graduations."""
    for step in (0.1, 0.2, 0.5, 1, 2, 5, 10):
        if span / step <= 8:
            return step
    return 10


def style_axes(ax):
    ax.set_facecolor(CANVAS)
    ax.set_aspect("equal", adjustable="datalim")
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.tick_params(colors=MUTED, labelsize=8, length=0)


# --------------------------------------------------------------------------
# Vue directe : un seul scan autour du robot, des anneaux de distance
# --------------------------------------------------------------------------
def draw_live(ax, scan, segments=None, use_intensity=True, limit=None):
    """Dessine le scan courant (repère du capteur). Retourne le rayon affiché."""
    ax.clear()
    style_axes(ax)
    if scan is not None and len(scan):
        max_r = float(np.max(scan.ranges))
        radius = limit or float(np.clip(np.ceil(max_r), 2, 12)) * 1.08
    else:
        radius = limit or 4.0
    step = nice_step(radius)
    for r in np.arange(step, radius * 1.5, step):
        ax.add_patch(Circle((0, 0), r, fill=False, ec=GRID, lw=1, zorder=1))
        ax.text(r * 0.707 + 0.03, r * 0.707 + 0.03, f"{r:g} m", color=MUTED, fontsize=7, zorder=1)
    ax.axhline(0, color=AXIS, lw=0.8, zorder=1)
    ax.axvline(0, color=AXIS, lw=0.8, zorder=1)
    if scan is not None and len(scan):
        pts = scan.points_local()
        colors = np.tile(matplotlib_rgb(LIVE), (len(pts), 1))
        if use_intensity:
            alpha = 0.35 + 0.65 * np.clip(scan.intensity / 255.0, 0, 1)
        else:
            alpha = np.ones(len(pts))
        colors = np.column_stack((colors, alpha))
        ax.scatter(pts[:, 0], pts[:, 1], s=7, c=colors, linewidths=0, zorder=3)
    if segments:
        ax.add_collection(LineCollection([[s.p1, s.p2] for s in segments], colors="#d9730d",
                                         linewidths=2, zorder=4))
    draw_robot(ax, 0.0, 0.0, 0.0, INK, size=9)
    ax.set_xlim(-radius, radius)
    ax.set_ylim(-radius, radius)
    return radius


def matplotlib_rgb(hex_color):
    from matplotlib.colors import to_rgb
    return np.array(to_rgb(hex_color))


def robot_marker(theta_deg):
    """Pointe de flèche dont la pointe regarde dans la direction theta (degrés, anti-horaire)."""
    shape = np.array([[1.0, 0.0], [-0.8, 0.65], [-0.4, 0.0], [-0.8, -0.65]])
    c, s = np.cos(np.radians(theta_deg)), np.sin(np.radians(theta_deg))
    rotated = shape @ np.array([[c, s], [-s, c]])          # rotation anti-horaire
    return [tuple(v) for v in rotated]


def draw_robot(ax, x, y, theta_deg, color, size=9):
    """Flèche orientée dans le sens du cap du robot."""
    return ax.plot([x], [y], marker=robot_marker(theta_deg), ms=size * 1.7, color=color,
                   markeredgecolor=CANVAS, markeredgewidth=1.0, linestyle="none", zorder=6)[0]


# --------------------------------------------------------------------------
# Vue carte : tous les scans, une couleur chacun, grille et segments
# --------------------------------------------------------------------------
def content_bounds(session, margin=0.4):
    """(xmin, xmax, ymin, ymax) englobant les scans visibles, ou None."""
    xs, ys = [], []
    for e in session.visible():
        pts = session.used(e).points_world(e.pose)
        xs.extend(pts[:, 0])
        ys.extend(pts[:, 1])
        xs.append(e.pose[0])
        ys.append(e.pose[1])
    if not xs:
        return None
    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


def draw_map(ax, session, selected=-1, show_points=True, show_grid=True, show_segments=True,
             view=None):
    """Dessine la carte. Retourne {nom_du_scan: (nuage, robot, étiquette)} pour les mises à jour rapides."""
    ax.clear()
    style_axes(ax)
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    ax.set_xlabel("x (m)", color=MUTED, fontsize=8)
    ax.set_ylabel("y (m)", color=MUTED, fontsize=8)

    grid = session.grid
    if show_grid and grid is not None:
        state = grid.classify()
        rgba = np.zeros(state.shape + (4,))
        rgba[state == 0] = (100 / 255, 116 / 255, 139 / 255, 0.13)       # libre : voile gris-bleu
        rgba[state == 1] = (26 / 255, 32 / 255, 39 / 255, 0.28)          # occupé : translucide
        ax.imshow(rgba, origin="lower", extent=grid.extent(), interpolation="nearest", zorder=1)

    if show_segments and session.vmap is not None and session.vmap.segments:
        ax.add_collection(LineCollection([[s.p1, s.p2] for s in session.vmap.segments],
                                         colors=INK, linewidths=1.3, zorder=3))

    artists = {}
    for k, e in enumerate(session.entries):
        if not e.visible:
            continue
        pts = session.used(e).points_world(e.pose)
        top = 5 if k == selected else 4
        cloud = None
        if show_points:
            cloud = ax.scatter(pts[:, 0], pts[:, 1], s=7 if k == selected else 5, color=e.color,
                               linewidths=0, zorder=top)
        robot = draw_robot(ax, e.pose[0], e.pose[1], e.pose[2], e.color,
                           size=12 if k == selected else 9)
        label = ax.annotate(e.name, (e.pose[0], e.pose[1]), xytext=(8, 7), textcoords="offset points",
                            color=e.color, fontsize=8, fontweight="bold", zorder=7)
        artists[e.name] = (cloud, robot, label)

    if view is None:
        view = content_bounds(session)
    if view is not None:
        xmin, xmax, ymin, ymax = view
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
    else:
        ax.set_xlim(-1, 7)
        ax.set_ylim(-1, 5)
    return artists


def move_scan_artists(artists, entry, filter_params):
    """Met à jour le nuage, le robot et l'étiquette d'un scan sans tout redessiner (glisser-déposer)."""
    cloud, robot, label = artists[entry.name]
    x, y, th = entry.pose
    if cloud is not None:
        cloud.set_offsets(transform_points(entry.used(filter_params).points_local(), entry.pose))
    robot.set_data([x], [y])
    robot.set_marker(robot_marker(th))
    label.set_position((x, y))


def render_map_png(session, selected=-1):
    """carte.png : la vue Carte complète, ajustée au contenu. Retourne les octets PNG."""
    from matplotlib.figure import Figure
    from lidar_session import fig_to_png_bytes
    fig = Figure(figsize=(8, 6), facecolor=CANVAS)
    ax = fig.add_subplot(111)
    draw_map(ax, session, selected)
    ax.set_title(f"Carte du relevé · {session.map_info}" if session.map_info else "Carte du relevé",
                 color=INK, fontsize=10)
    return fig_to_png_bytes(fig)


# --------------------------------------------------------------------------
# Vue « Grille 2D » : sans filtre à gauche, avec filtre à droite, séparés par un curseur
# --------------------------------------------------------------------------
GRID_OCC = (26 / 255, 32 / 255, 39 / 255, 1.0)           # cellule occupée : opaque, bien lisible
GRID_FREE = (100 / 255, 116 / 255, 139 / 255, 0.22)      # cellule libre : voile gris-bleu
WIPE_COLOR = LIVE


def grid_image(grid):
    """Image RGBA (origine en bas) : occupé foncé, libre en voile, inconnu transparent."""
    from gridmap import FREE, OCCUPIED
    state = grid.classify()
    rgba = np.zeros(state.shape + (4,))
    rgba[state == FREE] = GRID_FREE
    rgba[state == OCCUPIED] = GRID_OCC
    return rgba


def pair_bounds(session, pair, margin=0.4):
    """Cadrage conseillé : le contenu de la grille avec filtre (sans les rares points lointains non filtrés)."""
    xs, ys = [], []
    for e in session.visible():
        pts = session.used(e).points_world(e.pose)
        if len(pts):
            xs.extend((pts[:, 0].min(), pts[:, 0].max()))
            ys.extend((pts[:, 1].min(), pts[:, 1].max()))
        xs.append(e.pose[0])
        ys.append(e.pose[1])
    if not xs:
        x0, x1, y0, y1 = pair.raw.extent()
        return x0, x1, y0, y1
    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


def draw_grid(ax, session, pair, mode="cmp", wipe=0.5, view=None, selected=-1):
    """Dessine la grille 2D. mode : 'cmp' (comparaison), 'raw' (sans filtre) ou 'flt' (avec filtre).

    Retourne un dict d'éléments graphiques pour déplacer la séparation sans tout redessiner
    (voir `move_wipe`).
    """
    from matplotlib.patches import Rectangle
    ax.clear()
    style_axes(ax)
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    ax.set_xlabel("x (m)", color=MUTED, fontsize=8)
    ax.set_ylabel("y (m)", color=MUTED, fontsize=8)
    if pair is None:
        ax.set_xlim(-1, 7)
        ax.set_ylim(-1, 5)
        return {}
    xmin, xmax, ymin, ymax = view if view is not None else pair_bounds(session, pair)
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.apply_aspect()                       # l'aspect « égal » élargit une des limites : on relit les vraies
    xmin, xmax = ax.get_xlim()
    ymin, ymax = ax.get_ylim()

    extent = pair.raw.extent()
    art = {"mode": mode}
    if mode == "cmp":
        x_w = xmin + wipe * (xmax - xmin)
        big = 1e3
        left = Rectangle((xmin - big, ymin - big), x_w - xmin + big, 2 * big + (ymax - ymin), transform=ax.transData)
        right = Rectangle((x_w, ymin - big), xmax - x_w + big, 2 * big + (ymax - ymin), transform=ax.transData)
        im_raw = ax.imshow(grid_image(pair.raw), origin="lower", extent=extent, interpolation="nearest", zorder=1)
        im_flt = ax.imshow(grid_image(pair.flt), origin="lower", extent=extent, interpolation="nearest", zorder=1)
        im_raw.set_clip_path(left)
        im_flt.set_clip_path(right)
        line = ax.axvline(x_w, color=WIPE_COLOR, lw=2, zorder=8)
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        art.update(left=left, right=right, line=line, ax=ax, big=big)
        art["label_l"] = ax.text(0.01, 0.99, "Sans filtre", transform=ax.transAxes, ha="left", va="top",
                                 color=MUTED, fontsize=9, fontweight="bold", zorder=9)
        art["label_r"] = ax.text(0.99, 0.99, "Avec filtre", transform=ax.transAxes, ha="right", va="top",
                                 color=MUTED, fontsize=9, fontweight="bold", zorder=9)
    else:
        grid = pair.raw if mode == "raw" else pair.flt
        ax.imshow(grid_image(grid), origin="lower", extent=extent, interpolation="nearest", zorder=1)
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
    for k, e in enumerate(session.entries):
        if e.visible:
            draw_robot(ax, e.pose[0], e.pose[1], e.pose[2], e.color, size=9 if k == selected else 7)
    return art


def move_wipe(art, wipe):
    """Déplace la séparation (0 = tout à droite, 1 = tout à gauche) sans redessiner les images.

    La fraction se calcule sur les limites actuelles de l'axe : elle reste juste après un zoom ou un déplacement.
    """
    if art.get("mode") != "cmp":
        return
    ax = art["ax"]
    xmin, xmax = ax.get_xlim()
    ymin, ymax = ax.get_ylim()
    big = art["big"]
    x_w = xmin + wipe * (xmax - xmin)
    art["left"].set_xy((xmin - big, ymin - big))
    art["left"].set_width(x_w - xmin + big)
    art["left"].set_height(2 * big + (ymax - ymin))
    art["right"].set_xy((x_w, ymin - big))
    art["right"].set_width(xmax - x_w + big)
    art["right"].set_height(2 * big + (ymax - ymin))
    art["line"].set_xdata([x_w, x_w])


def wipe_fraction(art):
    """Position actuelle de la séparation, entre 0 et 1 (pour la redessiner au même endroit)."""
    ax = art["ax"]
    xmin, xmax = ax.get_xlim()
    x_w = art["line"].get_xdata()[0]
    return min(1.0, max(0.0, (x_w - xmin) / (xmax - xmin))) if xmax > xmin else 0.5


def render_comparison_png(session, pair):
    """Image « sans filtre | avec filtre » côte à côte, avec les chiffres en titre. Retourne les octets PNG."""
    from matplotlib.figure import Figure
    from lidar_session import fig_to_png_bytes
    fig = Figure(figsize=(13, 6), facecolor=CANVAS)
    axes = fig.subplots(1, 2)
    view = pair_bounds(session, pair)
    for ax, mode, title in zip(axes, ("raw", "flt"), ("Sans filtre", "Avec filtre")):
        draw_grid(ax, session, pair, mode, view=view)
        ax.set_title(title, color=INK, fontsize=10)
    st = pair.stats
    fig.suptitle(f"Grille d'occupation 2D · carreaux de {pair.cell_mm} mm · "
                 f"cellules occupées {st['raw']['occ']} → {st['flt']['occ']}, "
                 f"isolées {st['raw']['iso']} → {st['flt']['iso']}", fontsize=10)
    return fig_to_png_bytes(fig)
