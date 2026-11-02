"""Carte d'occupation 2D (grid map) en log-odds.

Principe (voir RAPPORT_Jour1.md) :
- chaque cellule stocke un log-odds l = ln(p / (1 - p)),  l = 0  <=> p = 0.5 (inconnu)
- pour chaque point du scan on trace un rayon du lidar jusqu'au point d'impact :
    * les cellules TRAVERSÉES sont probablement libres -> l += L_FREE  (négatif)
    * la cellule d'IMPACT est probablement occupée     -> l += L_OCC   (positif)
- on borne l dans [L_MIN, L_MAX] pour que la carte puisse "changer d'avis"
- option "durée de vie" : à chaque scan, l est multiplié par `forget` (< 1)
  pour oublier doucement les vieilles observations (objets qui bougent)
"""

import math

import numpy as np

from lidar_io import Scan

UNKNOWN, FREE, OCCUPIED = -1, 0, 1


def ray_cells(i0, j0, i1, j1):
    """Cellules traversées par le segment (i0,j0)->(i1,j1), SANS la dernière.

    Algorithme type DDA/Bresenham : on échantillonne n+1 points régulièrement
    espacés avec n = max(|di|, |dj|), puis on arrondit à la cellule la plus proche.
    """
    n = max(abs(i1 - i0), abs(j1 - j0))
    if n == 0:
        empty = np.zeros(0, dtype=int)
        return empty, empty
    ii = np.rint(np.linspace(i0, i1, n + 1)).astype(int)
    jj = np.rint(np.linspace(j0, j1, n + 1)).astype(int)
    return ii[:-1], jj[:-1]


class OccupancyGrid:
    def __init__(self, x_min, y_min, x_max, y_max, resolution=0.05,
                 l_occ=0.85, l_free=-0.40, l_min=-4.0, l_max=4.0, forget=1.0):
        self.x_min = float(x_min)
        self.y_min = float(y_min)
        self.resolution = float(resolution)
        self.nx = int(math.ceil((x_max - x_min) / resolution))
        self.ny = int(math.ceil((y_max - y_min) / resolution))
        self.l_occ = l_occ
        self.l_free = l_free
        self.l_min = l_min
        self.l_max = l_max
        self.forget = forget
        self.log_odds = np.zeros((self.ny, self.nx))     # indexé [j, i] = [ligne y, colonne x]
        self.seen = np.zeros((self.ny, self.nx), dtype=bool)   # cellule déjà observée ?
        self.n_updates = 0

    # ---- conversions ----------------------------------------------------
    def world_to_cell(self, x, y):
        i = int(math.floor((x - self.x_min) / self.resolution))
        j = int(math.floor((y - self.y_min) / self.resolution))
        return i, j

    def in_bounds(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny

    def cell_center(self, i, j):
        x = self.x_min + (i + 0.5) * self.resolution
        y = self.y_min + (j + 0.5) * self.resolution
        return x, y

    # ---- mise à jour ----------------------------------------------------
    def update(self, scan, pose, max_range=None):
        """Intègre un scan pris depuis `pose` = (x, y, theta_deg) dans la carte.

        Retourne (nb_cellules_libres_mises_à_jour, nb_cellules_occupées_mises_à_jour).
        """
        x0, y0, _theta = pose
        i0, j0 = self.world_to_cell(x0, y0)
        points_world = scan.points_world(pose)
        ranges = scan.ranges

        free_indices = []
        hit_indices = []
        for (px, py), r in zip(points_world, ranges):
            if max_range is not None and r > max_range:
                continue
            i1, j1 = self.world_to_cell(px, py)

            # cellules traversées (on ignore celles hors de la carte)
            ii, jj = ray_cells(i0, j0, i1, j1)
            inside = (ii >= 0) & (ii < self.nx) & (jj >= 0) & (jj < self.ny)
            free_indices.append(jj[inside] * self.nx + ii[inside])

            # cellule d'impact
            if self.in_bounds(i1, j1):
                hit_indices.append(j1 * self.nx + i1)

        if free_indices:
            free_flat = np.unique(np.concatenate(free_indices))
        else:
            free_flat = np.zeros(0, dtype=int)
        hit_flat = np.unique(np.array(hit_indices, dtype=int))

        # une cellule touchée par un rayon ET traversée par un autre (rayon rasant)
        # reste occupée : l'impact l'emporte
        free_only = np.setdiff1d(free_flat, hit_flat)

        flat = self.log_odds.reshape(-1)
        seen_flat = self.seen.reshape(-1)
        if self.forget < 1.0:
            flat *= self.forget
        flat[free_only] += self.l_free
        flat[hit_flat] += self.l_occ
        np.clip(flat, self.l_min, self.l_max, out=flat)
        seen_flat[free_only] = True
        seen_flat[hit_flat] = True

        self.n_updates += 1
        return len(free_only), len(hit_flat)

    # ---- lecture --------------------------------------------------------
    def probability(self):
        """Probabilité d'occupation p dans [0, 1] pour chaque cellule."""
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def classify(self, p_free=0.45, p_occ=0.60):
        """Carte discrète : UNKNOWN (-1), FREE (0), OCCUPIED (1)."""
        prob = self.probability()
        result = np.full(prob.shape, UNKNOWN, dtype=int)
        result[self.seen & (prob <= p_free)] = FREE
        result[self.seen & (prob >= p_occ)] = OCCUPIED
        return result

    def extent(self):
        """(x_min, x_max, y_min, y_max) pour imshow(extent=..., origin='lower')."""
        x_max = self.x_min + self.nx * self.resolution
        y_max = self.y_min + self.ny * self.resolution
        return (self.x_min, x_max, self.y_min, y_max)

    def save_csv(self, path):
        """Exporte la carte discrète (1=occupé, 0=libre, -1=inconnu), ligne du haut en premier."""
        grid = self.classify()
        flipped = np.flipud(grid)
        np.savetxt(path, flipped, fmt="%d", delimiter=",")

    def save_npz(self, path):
        np.savez_compressed(path, log_odds=self.log_odds, seen=self.seen,
                            x_min=self.x_min, y_min=self.y_min,
                            resolution=self.resolution,
                            params=np.array([self.l_occ, self.l_free, self.l_min,
                                             self.l_max, self.forget]))

    @classmethod
    def load_npz(cls, path):
        data = np.load(path)
        res = float(data["resolution"])
        ny, nx = data["log_odds"].shape
        l_occ, l_free, l_min, l_max, forget = (float(v) for v in data["params"])
        grid = cls(float(data["x_min"]), float(data["y_min"]),
                   float(data["x_min"]) + nx * res, float(data["y_min"]) + ny * res,
                   resolution=res, l_occ=l_occ, l_free=l_free, l_min=l_min,
                   l_max=l_max, forget=forget)
        grid.log_odds = data["log_odds"].copy()
        grid.seen = data["seen"].copy()
        return grid


def grid_from_scan(scan, pose, resolution=0.05, margin=0.5, **kwargs):
    """Crée une NOUVELLE carte qui contient exactement le scan, puis l'y intègre."""
    points_world = scan.points_world(pose)
    x0, y0, _ = pose
    all_x = np.append(points_world[:, 0], x0)
    all_y = np.append(points_world[:, 1], y0)
    grid = OccupancyGrid(all_x.min() - margin, all_y.min() - margin,
                         all_x.max() + margin, all_y.max() + margin,
                         resolution=resolution, **kwargs)
    grid.update(scan, pose)
    return grid


def grid_for_scans(scans, poses, resolution=0.05, margin=0.5, **kwargs):
    """Crée une carte vide assez grande pour contenir tous les scans placés."""
    xs = []
    ys = []
    for scan in scans:
        pose = poses[scan.name]
        pts = scan.points_world(pose)
        xs.extend(pts[:, 0])
        ys.extend(pts[:, 1])
        xs.append(pose[0])
        ys.append(pose[1])
    return OccupancyGrid(min(xs) - margin, min(ys) - margin,
                         max(xs) + margin, max(ys) + margin,
                         resolution=resolution, **kwargs)


def plot_grid(grid, ax, scans_poses=None):
    """Dessine la carte : gris = inconnu, blanc = libre, noir = occupé."""
    state = grid.classify()
    image = np.full(state.shape, 0.55)       # inconnu : gris
    image[state == FREE] = 1.0
    image[state == OCCUPIED] = 0.0
    x_min, x_max, y_min, y_max = grid.extent()
    ax.imshow(image, origin="lower", cmap="gray", vmin=0, vmax=1,
              extent=(x_min, x_max, y_min, y_max), interpolation="nearest")
    if scans_poses:
        for x, y, theta in scans_poses:
            ax.plot(x, y, "r+", ms=10)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
