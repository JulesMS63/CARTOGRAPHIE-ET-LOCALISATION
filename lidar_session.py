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
from gridmap import grid_for_scans
from lidar_io import Scan, _make_scan, load_scans, save_scan
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


@dataclass
class Entry:
    scan: Scan
    pose: list
    color: str
    visible: bool = True
    _segments: list = field(default=None, repr=False)

    @property
    def name(self):
        return self.scan.name

    def local_segments(self):
        if self._segments is None:
            self._segments = extract_segments(self.scan.points_local())
        return self._segments


@dataclass
class MapParams:
    resolution: float = 0.05
    forget: float = 1.0
    ang_tol_deg: float = 10.0
    dist_tol: float = 0.06
    gap_tol: float = 0.15
    min_weight: float = 1.0


class Session:
    def __init__(self):
        self.entries = []
        self.counters = {}
        self.color_index = 0
        self.grid = None
        self.vmap = None
        self.map_info = ""

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
            pose = (poses or {}).get(scan.name, (0.0, 0.0, 0.0))
            named = Scan(self._unique_name(scan.name), scan.angles, scan.ranges, scan.intensity)
            self.entries.append(Entry(named, [float(p) for p in pose], self._next_color()))
            added += 1
        return added

    def import_files(self, paths):
        """Importe des .csv (formats ESP32, maison, RobotControl, MapTool), un projet.json ou un
        poses.json (qui s'applique aux scans déjà présents). Retourne le nombre de scans ajoutés."""
        added = 0
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
        vis = self.visible()
        if not vis:
            self.grid = self.vmap = None
            self.map_info = ""
            return
        poses = {e.name: tuple(e.pose) for e in vis}
        scans = [e.scan for e in vis]
        grid = grid_for_scans(scans, poses, resolution=params.resolution, forget=params.forget)
        for scan in scans:
            grid.update(scan, poses[scan.name])
        vmap = VectorMap(ang_tol_deg=params.ang_tol_deg, dist_tol=params.dist_tol,
                         gap_tol=params.gap_tol)
        for e in vis:
            vmap.add_segments([segment_to_world(s, e.pose) for s in e.local_segments()])
        vmap.prune(params.min_weight)
        self.grid, self.vmap = grid, vmap
        self.map_info = (f"{len(vis)} scan(s) · grille {grid.nx}×{grid.ny} · "
                         f"{len(vmap.segments)} segments")

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

    def export_folder(self, folder, png_bytes=None):
        """Écrit tout le relevé dans un dossier (lisible par gridmap_tool.py / vectormap_tool.py)."""
        folder = Path(folder)
        (folder / "scans").mkdir(parents=True, exist_ok=True)
        for e in self.entries:
            save_scan(e.scan, folder / "scans" / f"{e.name}.csv")
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

    def export_zip(self, path, png_bytes=None):
        """Même contenu que export_folder, dans un .zip (même organisation que la page web)."""
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            self.export_folder(tmp, png_bytes)
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
