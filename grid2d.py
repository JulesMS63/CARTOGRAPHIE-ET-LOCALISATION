"""Grille d'occupation 2D « sans filtre / avec filtre » : de quoi juger l'effet du filtre des points isolés.

On construit DEUX grilles sur exactement la même étendue (calculée sur les scans bruts, la plus grande) :
    raw : tous les points mesurés
    flt : seulement les points gardés par le filtre (scanfilter.py)
Elles se superposent donc cellule par cellule. `grid_stats` compte les cellules occupées et, parmi elles,
les cellules ISOLÉES (aucune des 8 cellules voisines n'est occupée) : un bon filtre fait tomber les cellules
isolées sans réduire fortement le nombre de cellules des murs.

La taille des carreaux se donne en millimètres, en entier (`cell_mm`).
"""

import math
from dataclasses import dataclass

import numpy as np

from gridmap import FREE, OCCUPIED, OccupancyGrid, grid_for_scans

CELL_MM_MIN, CELL_MM_MAX = 10, 500
MAX_CELLS = 4_000_000            # au-delà, on agrandit les carreaux (sinon le calcul devient très long)


def clamp_cell_mm(value, default=50):
    """Entier de 10 à 500 mm (une valeur vide ou illisible donne `default`)."""
    try:
        mm = int(round(float(value)))
    except (TypeError, ValueError):
        return default
    return min(CELL_MM_MAX, max(CELL_MM_MIN, mm))


def grid_stats(grid):
    """{'occ': cellules occupées, 'iso': dont isolées, 'free': cellules libres}."""
    state = grid.classify()
    occ = state == OCCUPIED
    padded = np.pad(occ, 1)
    neighbours = np.zeros(occ.shape, dtype=int)
    for dj in (-1, 0, 1):
        for di in (-1, 0, 1):
            if di or dj:
                neighbours += padded[1 + dj:1 + dj + occ.shape[0], 1 + di:1 + di + occ.shape[1]]
    return {"occ": int(occ.sum()), "iso": int((occ & (neighbours == 0)).sum()),
            "free": int((state == FREE).sum())}


@dataclass
class GridPair:
    raw: OccupancyGrid
    flt: OccupancyGrid
    cell_mm: int                 # taille réellement utilisée (peut être plus grande que demandée si la zone est énorme)
    stats: dict                  # {'raw': {...}, 'flt': {...}, 'ignored': n, 'total': n}
    capped: bool = False


def build_grid_pair(entries, used_of, cell_mm=50, forget=1.0):
    """entries : objets avec .name, .scan (brut) et .pose ; used_of(entry) -> scan filtré.

    Retourne un GridPair, ou None s'il n'y a aucun scan.
    """
    if not entries:
        return None
    cell_mm = clamp_cell_mm(cell_mm)
    poses = {e.name: tuple(e.pose) for e in entries}
    raws = [e.scan for e in entries]
    capped = False
    raw = grid_for_scans(raws, poses, resolution=cell_mm / 1000.0, forget=forget)
    if raw.nx * raw.ny > MAX_CELLS:
        cell_mm = min(CELL_MM_MAX, math.ceil(cell_mm * math.sqrt(raw.nx * raw.ny / MAX_CELLS)))
        capped = True
        raw = grid_for_scans(raws, poses, resolution=cell_mm / 1000.0, forget=forget)
    res = raw.resolution
    flt = OccupancyGrid(raw.x_min, raw.y_min, raw.x_min + (raw.nx - 0.5) * res,
                        raw.y_min + (raw.ny - 0.5) * res, resolution=res, forget=forget)
    assert (flt.nx, flt.ny) == (raw.nx, raw.ny)
    used = []
    for e in entries:
        raw.update(e.scan, poses[e.name])
        scan = used_of(e)
        flt.update(scan, poses[e.name])
        used.append(scan)
    total = sum(len(s) for s in raws)
    ignored = total - sum(len(s) for s in used)
    stats = {"raw": grid_stats(raw), "flt": grid_stats(flt), "ignored": int(ignored), "total": int(total)}
    return GridPair(raw, flt, cell_mm, stats, capped)


def stats_table(stats):
    """Le tableau de comparaison en texte (rapport, console)."""
    pct = round(100 * stats["ignored"] / stats["total"]) if stats["total"] else 0
    rows = [("Cellules occupées", stats["raw"]["occ"], stats["flt"]["occ"]),
            ("dont isolées", stats["raw"]["iso"], stats["flt"]["iso"]),
            ("Points ignorés", 0, f"{stats['ignored']} ({pct} %)")]
    lines = [f"{'':22s}{'Sans filtre':>14s}{'Avec filtre':>16s}"]
    lines += [f"{name:22s}{a!s:>14s}{b!s:>16s}" for name, a, b in rows]
    return "\n".join(lines)
