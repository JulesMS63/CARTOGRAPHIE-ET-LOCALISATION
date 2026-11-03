"""Données d'une session de relevé : scans capturés, poses, cartes, export.

Sans interface graphique (donc testable). L'application `lidar_app.py` s'appuie dessus.

Conventions (identiques aux autres modules) : pose = (x en m, y en m, theta en degrés),
X vers la droite, Y vers le haut, angle positif dans le sens anti-horaire.
"""

import io
import json
import math
import re
import zipfile
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from geometry import wrap_deg
from grid2d import build_grid_pair
from gridmap import grid_for_scans
from lidar_io import Scan, _make_scan, load_scans, save_scan
from scanfilter import DEFAULT_BREAK_MIN, DEFAULT_CONFIRM_TOL, DEFAULT_MIN_POINTS, DEFAULT_MIN_RANGE, confirm_scans, filter_isolated
from segments import extract_segments
from vectormap import VectorMap, segment_to_world



def scan_color(i):
    """Couleur n° i, sans limite : on avance de 137,5° sur la roue des teintes (angle d'or),
    donc deux scans consécutifs ont toujours des couleurs bien différentes. Même règle que la page web."""
    import colorsys
    hue = ((220 + i * 137.508) % 360) / 360.0
    r, g, b = colorsys.hls_to_rgb(hue, 0.46, 0.78)
    return "#{:02x}{:02x}{:02x}".format(round(r * 255), round(g * 255), round(b * 255))


def compose_pose(base, delta):
    """Pose absolue obtenue en partant de `base` et en se déplaçant de `delta`.

    delta = (avant, gauche, rotation) exprimé dans le repère du robot à la pose `base` :
    avant = le long de son cap, gauche = à sa gauche, rotation en degrés (anti-horaire).
    """
    bx, by, bth = base
    dx, dy, dth = delta
    c, s = math.cos(math.radians(bth)), math.sin(math.radians(bth))
    return (bx + c * dx - s * dy, by + s * dx + c * dy, wrap_deg(bth + dth))


@dataclass(frozen=True)
class FilterParams:
    """Réglages du filtre des points isolés (voir scanfilter.py)."""
    enabled: bool = True
    min_points: int = DEFAULT_MIN_POINTS
    min_range: float = DEFAULT_MIN_RANGE
    break_min: float = DEFAULT_BREAK_MIN
    confirm: bool = False               # exiger un 2e scan : ignorer ce qu'un seul scan voit
    confirm_tol: float = DEFAULT_CONFIRM_TOL

    def kwargs(self):
        return {"min_points": self.min_points, "min_range": self.min_range, "break_min": self.break_min}


@dataclass
class Entry:
    scan: Scan                      # scan brut, jamais modifié
    pose: list
    color: str
    visible: bool = True
    _used: tuple = field(default=None, repr=False)
    _segments: tuple = field(default=None, repr=False)

    @property
    def name(self):
        return self.scan.name

    def used(self, fp):
        """Le scan à utiliser pour les cartes et le dessin : filtré si `fp.enabled`, sinon le brut."""
        if not fp.enabled:
            return self.scan
        if self._used is None or self._used[0] != fp:
            self._used = (fp, filter_isolated(self.scan, **fp.kwargs()))
        return self._used[1]

    def ignored(self, fp):
        return len(self.scan) - len(self.used(fp))

    def local_segments(self, used=None):
        """Segments (repère du capteur) du scan `used` (par défaut le scan brut), mis en cache."""
        used = self.scan if used is None else used
        if self._segments is None or self._segments[0] is not used:
            self._segments = (used, extract_segments(used.points_local()))
        return self._segments[1]


@dataclass
class MapParams:
    resolution: float = 0.05
    forget: float = 1.0
    ang_tol_deg: float = 10.0
    dist_tol: float = 0.06
    gap_tol: float = 0.15
    min_weight: float = 1.0
    clean: bool = True              # ignorer les points isolés
    min_points: int = DEFAULT_MIN_POINTS
    min_range: float = DEFAULT_MIN_RANGE
    break_min: float = DEFAULT_BREAK_MIN
    confirm: bool = False
    confirm_tol: float = DEFAULT_CONFIRM_TOL

    @property
    def cell_mm(self):
        return int(round(self.resolution * 1000))

    def filter(self):
        return FilterParams(self.clean, max(1, int(self.min_points)), max(0.0, float(self.min_range)),
                            max(0.02, float(self.break_min)), bool(self.confirm),
                            max(0.02, float(self.confirm_tol)))


class Session:
    def __init__(self):
        self.entries = []
        self.counters = {}
        self.color_index = 0
        self.grid = None
        self.vmap = None
        self.map_info = ""
        self.filter = FilterParams()
        self._confirmed = (None, {})        # (clé, {nom: scan confirmé}) : recalculé seulement si quelque chose change
        self.last_import_posed = 0

    # ---- scan utilisé pour les cartes ----------------------------------
    def used(self, entry):
        """Scan de `entry` tel qu'il compte dans les cartes : filtré, puis confirmé par un autre scan si demandé."""
        base = entry.used(self.filter)
        if not (self.filter.enabled and self.filter.confirm and entry.visible):
            return base
        vis = self.visible()
        if len(vis) < 2:
            return base
        key = (self.filter, tuple((e.name, tuple(round(float(v), 4) for v in e.pose),
                                   len(e.used(self.filter))) for e in vis))
        if self._confirmed[0] != key:
            bases = [e.used(self.filter) for e in vis]
            poses = {e.name: tuple(e.pose) for e in vis}
            done = confirm_scans(bases, poses, self.filter.confirm_tol)
            self._confirmed = (key, {e.name: s for e, s in zip(vis, done)})
        return self._confirmed[1].get(entry.name, base)

    def ignored_count(self, entries=None):
        """Nombre de points ignorés (filtre + confirmation) dans les scans donnés (par défaut : visibles)."""
        if not self.filter.enabled:
            return 0
        entries = self.visible() if entries is None else entries
        return sum(len(e.scan) - len(self.used(e)) for e in entries)

    # ---- scans ---------------------------------------------------------
    def _next_color(self):
        color = scan_color(self.color_index)
        self.color_index += 1
        return color

    def _unique_name(self, name):
        names = {e.name for e in self.entries}
        if name not in names:
            return name
        k = 2
        while f"{name}_{k}" in names:
            k += 1
        return f"{name}_{k}"

    def last_pose(self):
        return tuple(self.entries[-1].pose) if self.entries else None

    def pose_from_input(self, mode, values):
        """mode 'abs' : values = (x, y, theta) depuis l'origine ; 'rel' : (avant, gauche, rotation)
        depuis le dernier scan. Le premier scan donne toujours l'origine telle que saisie."""
        base = self.last_pose()
        if mode == "rel" and base is not None:
            return compose_pose(base, values)
        return tuple(float(v) for v in values)

    def capture(self, scan, prefix, pose):
        prefix = re.sub(r"[^\w\-]+", "_", prefix or "scan")
        k = self.counters.get(prefix, -1) + 1
        self.counters[prefix] = k
        named = Scan(self._unique_name(f"{prefix}_{k:03d}"), scan.angles.copy(),
                     scan.ranges.copy(), scan.intensity.copy())
        entry = Entry(named, [float(p) for p in pose], self._next_color())
        self.entries.append(entry)
        return entry

    def add_scans(self, scans, poses=None):
        added = 0
        for scan in scans:
            pose = (poses or {}).get(scan.name, scan.pose or (0.0, 0.0, 0.0))     # sinon : position écrite dans le .csv
            self.last_import_posed += int(scan.pose is not None and scan.name not in (poses or {}))
            named = Scan(self._unique_name(scan.name), scan.angles, scan.ranges, scan.intensity)
            self.entries.append(Entry(named, [float(p) for p in pose], self._next_color()))
            added += 1
        return added

    def import_files(self, paths):
        """Importe des .csv (formats ESP32, maison, RobotControl, MapTool), un projet.json ou un
        poses.json (qui s'applique aux scans déjà présents). Retourne le nombre de scans ajoutés."""
        added = 0
        self.last_import_posed = 0
        for path in map(Path, paths):
            if path.suffix.lower() == ".json":
                data = json.loads(path.read_text(encoding="utf-8"))
                if isinstance(data, dict) and "scans" in data:
                    added += self._load_project_dict(data)
                else:
                    poses = {n: tuple(float(v) for v in p) for n, p in data.items()}
                    for e in self.entries:
                        if e.name in poses:
                            e.pose = list(poses[e.name])
            else:
                added += self.add_scans(load_scans(path))
        return added

    def _load_project_dict(self, data):
        count = 0
        for sc in data["scans"]:
            scan = _make_scan(sc["name"], sc["angles_deg"],
                              np.asarray(sc["distances_mm"], dtype=float) / 1000.0, sc["intensity"])
            self.add_scans([scan], {scan.name: sc["pose"]})
            count += 1
        return count

    def remove(self, index):
        del self.entries[index]

    def clear(self):
        self.entries.clear()
        self.counters.clear()
        self.color_index = 0
        self.grid = self.vmap = None
        self.map_info = ""

    def visible(self):
        return [e for e in self.entries if e.visible]

    def poses_dict(self):
        return {e.name: tuple(e.pose) for e in self.entries}

    # ---- cartes --------------------------------------------------------
    def rebuild(self, params=None):
        """Recalcule la grille d'occupation et la carte vectorielle à partir des scans visibles."""
        params = params or MapParams()
        self.filter = params.filter()
        vis = self.visible()
        if not vis:
            self.grid = self.vmap = None
            self.map_info = ""
            return
        poses = {e.name: tuple(e.pose) for e in vis}
        scans = [self.used(e) for e in vis]
        grid = grid_for_scans(scans, poses, resolution=params.resolution, forget=params.forget)
        for scan in scans:
            grid.update(scan, poses[scan.name])
        vmap = VectorMap(ang_tol_deg=params.ang_tol_deg, dist_tol=params.dist_tol,
                         gap_tol=params.gap_tol)
        for e in vis:
            vmap.add_segments([segment_to_world(s, e.pose) for s in e.local_segments(self.used(e))])
        vmap.prune(params.min_weight)
        self.grid, self.vmap = grid, vmap
        self.map_info = (f"{len(vis)} scan(s) · grille {grid.nx}×{grid.ny} · "
                         f"{len(vmap.segments)} segments")
        if self.filter.enabled:
            ignored = self.ignored_count(vis)
            self.map_info += f" · {ignored} point{'s' if ignored > 1 else ''} ignoré{'s' if ignored > 1 else ''}"

    # ---- grille 2D : sans filtre / avec filtre --------------------------
    def grid_pair(self, cell_mm=50, forget=1.0):
        """GridPair (voir grid2d.py) pour les scans visibles, mis en cache tant que rien ne change."""
        vis = self.visible()
        key = (cell_mm, forget, self.filter, tuple((e.name, tuple(round(float(v), 4) for v in e.pose), len(e.scan))
                                                    for e in vis))
        if getattr(self, "_pair", (None, None))[0] != key:
            self._pair = (key, build_grid_pair(vis, self.used, cell_mm, forget))
        return self._pair[1]

    # ---- projet / export -----------------------------------------------
    def poses_json(self):
        data = {e.name: [round(float(v), 4) for v in e.pose] for e in self.entries}
        return json.dumps(data, indent=2, ensure_ascii=False) + "\n"

    def project_dict(self):
        return {"version": 1, "scans": [{
            "name": e.name, "pose": [float(v) for v in e.pose],
            "angles_deg": [round(float(a), 3) for a in np.degrees(e.scan.angles)],
            "distances_mm": [int(round(float(r) * 1000)) for r in e.scan.ranges],
            "intensity": [int(i) for i in e.scan.intensity],
        } for e in self.entries]}

    def export_folder(self, folder, png_bytes=None, extra_files=None):
        """Écrit tout le relevé dans un dossier (lisible par gridmap_tool.py / vectormap_tool.py)."""
        folder = Path(folder)
        (folder / "scans").mkdir(parents=True, exist_ok=True)
        for e in self.entries:
            save_scan(self.used(e), folder / "scans" / f"{e.name}.csv", pose=e.pose)    # filtré, avec sa position ; projet.json reste brut
        (folder / "poses.json").write_text(self.poses_json(), encoding="utf-8")
        (folder / "projet.json").write_text(json.dumps(self.project_dict()) + "\n", encoding="utf-8")
        self.rebuild_if_needed()
        if self.vmap is not None:
            self.vmap.save(folder / "carte_vectorielle.txt")
        if self.grid is not None:
            self.grid.save_csv(folder / "grille.csv")
            self.grid.save_npz(folder / "grille.npz")
        if png_bytes:
            (folder / "carte.png").write_bytes(png_bytes)
        for name, data in (extra_files or {}).items():          # ex. grille2d_comparaison.png
            (folder / name).write_bytes(data)

    def export_zip(self, path, png_bytes=None, extra_files=None):
        """Même contenu que export_folder, dans un .zip (même organisation que la page web)."""
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            self.export_folder(tmp, png_bytes, extra_files)
            with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as zf:
                for f in sorted(Path(tmp).rglob("*")):
                    if f.is_file() and f.suffix != ".npz":
                        zf.write(f, f.relative_to(tmp).as_posix())

    def rebuild_if_needed(self):
        if self.entries and self.grid is None:
            self.rebuild()


def fig_to_png_bytes(fig, dpi=130):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi, bbox_inches="tight")
    return buf.getvalue()
