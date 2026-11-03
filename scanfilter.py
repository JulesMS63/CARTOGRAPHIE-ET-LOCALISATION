"""Filtre des points isolés d'un scan.

Principe : un vrai obstacle (mur, meuble, pied de table) renvoie plusieurs points voisins ; un point parasite
(reflet, poussière, bruit près du capteur) est seul ou presque. On procède en deux temps :

1. Distance minimale : les points plus proches que `min_range` du capteur sont retirés (corps du robot, câbles).
2. Groupes : les points du balayage (triés par angle) sont reliés à leurs voisins quand l'écart entre eux
   ne dépasse pas   max(break_min, break_k * r_min * k * delta_theta)
   (r_min = la plus petite des deux distances, delta_theta = pas angulaire médian, k = nombre de pas qui
   séparent les deux points). Un point est relié à ses `reach` voisins suivants (2 par défaut) : un point
   parasite glissé au milieu d'un mur ne coupe donc pas le mur en deux. Les points reliés forment un groupe ;
   un groupe de moins de `min_points` points est supprimé. Le balayage est circulaire : le dernier point
   est voisin du premier.

3. (Option) Confirmation par un autre scan : `confirm_scans` ne garde un point que si au moins un AUTRE scan voit
   un point à moins de `tol` mètres, dans le repère de la carte. Ce qui n'est vu qu'une fois (personne qui
   passe, reflet) disparaît ; cela suppose des poses assez justes (tol plus grand que l'erreur de placement).

Le même algorithme est écrit en JavaScript dans lidar_web.html (filterScan, confirmScans) ; les deux sont
comparés dans les tests (tests/test_scanfilter.py).
"""

import numpy as np

from geometry import polar_to_cartesian
from lidar_io import Scan

DEFAULT_MIN_POINTS = 8        # un groupe plus petit est supprimé
DEFAULT_MIN_RANGE = 0.20      # m : en dessous, point collé au capteur
DEFAULT_BREAK_MIN = 0.10      # m : écart toujours toléré entre deux points voisins
DEFAULT_BREAK_K = 4.0         # l'écart toléré grandit avec la distance
DEFAULT_REACH = 2             # un point est relié à ses 2 voisins suivants
DEFAULT_CONFIRM_TOL = 0.10    # m : écart toléré entre deux scans pour confirmer un point


def isolated_mask(scan, min_points=DEFAULT_MIN_POINTS, min_range=DEFAULT_MIN_RANGE,
                  break_min=DEFAULT_BREAK_MIN, break_k=DEFAULT_BREAK_K, reach=DEFAULT_REACH):
    """Tableau de booléens (un par point du scan) : True = point conservé."""
    n = len(scan)
    keep = np.zeros(n, dtype=bool)
    idx = np.flatnonzero(scan.ranges >= min_range)
    m = len(idx)
    if m == 0:
        return keep
    if m < 2:
        keep[idx] = m >= min_points
        return keep

    angles = scan.angles[idx]
    ranges = scan.ranges[idx]
    points = polar_to_cartesian(angles, ranges)
    d_theta = float(np.median(np.abs(np.diff(angles))))

    parent = list(range(m))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    base = np.arange(m)
    for step in range(1, min(reach, m - 1) + 1):
        nxt = (base + step) % m
        gap = np.hypot(*(points[nxt] - points).T)
        limit = np.maximum(break_min, break_k * np.minimum(ranges, ranges[nxt]) * step * d_theta)
        for i in np.flatnonzero(gap <= limit):
            a, b = find(int(i)), find(int(nxt[i]))
            if a != b:
                parent[a] = b

    roots = np.array([find(i) for i in range(m)])
    sizes = np.bincount(roots, minlength=m)
    keep[idx] = sizes[roots] >= min_points
    return keep


def filter_isolated(scan, **kwargs):
    """Copie du scan sans les points isolés (le scan d'origine n'est pas modifié)."""
    keep = isolated_mask(scan, **kwargs)
    return Scan(scan.name, scan.angles[keep], scan.ranges[keep], scan.intensity[keep], scan.pose)


def confirm_scans(scans, poses, tol=DEFAULT_CONFIRM_TOL):
    """Retourne, pour chaque scan, une copie où ne restent que les points vus aussi par un autre scan.

    `poses` : {nom_du_scan: (x, y, theta_deg)}. Avec un seul scan, rien n'est retiré.
    """
    if len(scans) < 2:
        return list(scans)
    world = [scan.points_world(poses[scan.name]) for scan in scans]
    result = []
    for k, scan in enumerate(scans):
        others = [w for j, w in enumerate(world) if j != k and len(w)]
        keep = np.zeros(len(scan), dtype=bool)
        if others and len(scan):
            q = np.concatenate(others)
            mine = world[k]
            for start in range(0, len(mine), 128):          # par paquets : pas de grosse matrice en mémoire
                chunk = mine[start:start + 128]
                d2 = ((chunk[:, None, :] - q[None, :, :]) ** 2).sum(axis=2)
                keep[start:start + 128] = d2.min(axis=1) <= tol * tol
        result.append(Scan(scan.name, scan.angles[keep], scan.ranges[keep], scan.intensity[keep],
                           scan.pose))
    return result
