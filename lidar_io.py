"""Lecture / écriture des scans lidar.

Formats reconnus automatiquement (d'après l'en-tête du fichier) :

1. Sortie ESP32 `printScanCSV` de la librairie LD06 :
       N,Angle(°),Distance(mm),Intensity[,x(mm),y(mm)]
   Un fichier peut contenir PLUSIEURS scans, séparés par une ligne vide.
2. Notre format de sauvegarde :
       angle_deg,distance_mm,intensity
3. Export RobotControl :
       angle_deg,distance_m,intensity
4. Export MapTool :
       angle_deg,r_m,x_rel_m,y_rel_m      (r_m = 0 -> pas d'impact)

Position du scan : nos fichiers .csv commencent par une ligne de commentaire
       # pose: x=1.2000 y=0.5000 theta=30.0000        (x, y en m ; theta en degrés)
La position suit donc le fichier : elle est relue par `load_scans` (attribut `scan.pose`).
Les autres lecteurs voient simplement un commentaire.

Dans tous les cas on recalcule x et y NOUS-MÊMES à partir de (angle, distance),
on n'utilise pas les colonnes x/y de la librairie (voir RAPPORT : signe de y).
"""

import json
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from geometry import polar_to_cartesian, transform_points  # noqa: E402

MIN_RANGE_M = 0.02    # en dessous : mesure invalide
MAX_RANGE_M = 12.0    # portée max du LD06


@dataclass
class Scan:
    """Un tour complet du lidar, en coordonnées polaires dans le repère capteur."""

    name: str
    angles: np.ndarray       # radians, sens anti-horaire (math)
    ranges: np.ndarray       # mètres
    intensity: np.ndarray    # 0..255
    pose: tuple = None       # (x_m, y_m, theta_deg) relue dans le fichier .csv, ou None si inconnue

    def __len__(self):
        return len(self.ranges)

    def points_local(self):
        """Nuage de points (N, 2) dans le repère du lidar."""
        return polar_to_cartesian(self.angles, self.ranges)

    def points_world(self, pose):
        """Nuage de points (N, 2) dans le repère monde pour une pose donnée."""
        local = self.points_local()
        return transform_points(local, pose)


def _make_scan(name, angles_deg, ranges_m, intensity):
    """Construit un Scan en retirant les mesures invalides et en triant par angle."""
    angles_deg = np.asarray(angles_deg, dtype=float)
    ranges_m = np.asarray(ranges_m, dtype=float)
    intensity = np.asarray(intensity, dtype=float)

    valid = (ranges_m >= MIN_RANGE_M) & (ranges_m <= MAX_RANGE_M)
    angles_deg = angles_deg[valid]
    ranges_m = ranges_m[valid]
    intensity = intensity[valid]

    angles_rad = np.radians(angles_deg % 360.0)
    order = np.argsort(angles_rad)
    scan = Scan(name=name,
                angles=angles_rad[order],
                ranges=ranges_m[order],
                intensity=intensity[order])
    return scan


_POSE_RE = re.compile(r"^#\s*pose\b", re.IGNORECASE)


def parse_pose_comment(line):
    """'# pose: x=1.2 y=0.5 theta=30' -> (1.2, 0.5, 30.0), ou None si ce n'est pas une ligne de pose."""
    if not _POSE_RE.match(line):
        return None
    values = {}
    for key in ("x", "y", "theta"):
        m = re.search(rf"\b{key}\s*=\s*(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)", line, re.IGNORECASE)
        if not m:
            return None
        values[key] = float(m.group(1))
    return values["x"], values["y"], values["theta"]


def pose_comment(pose):
    """Ligne de commentaire à mettre en tête d'un fichier .csv pour y conserver la position du scan."""
    x, y, theta = pose
    return f"# pose: x={x:.4f} y={y:.4f} theta={theta:.4f}"


def _is_number(text):
    try:
        float(text)
        return True
    except ValueError:
        return False


def load_scans(path):
    """Charge tous les scans d'un fichier. Retourne une liste de Scan."""
    path = Path(path)
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()

    fmt = "esp32"            # format par défaut si pas d'en-tête
    blocks = []              # liste de listes de lignes numériques
    block_poses = []         # une pose (ou None) par bloc
    current = []
    seen_pose = None

    for raw in lines:
        line = raw.strip()
        if line == "":
            if current:
                blocks.append(current)
                block_poses.append(seen_pose)
                current = []
                seen_pose = None
            continue
        if line.startswith("#"):
            found = parse_pose_comment(line)
            if found is not None:
                seen_pose = found
            continue

        fields = [f.strip() for f in line.split(",")]
        if not _is_number(fields[0]):
            # ligne d'en-tête : on détecte le format
            lowered = line.lower()
            if lowered.startswith("n,angle"):
                fmt = "esp32"
            elif "distance_mm" in lowered:
                fmt = "own"
            elif "distance_m" in lowered:
                fmt = "robotcontrol"
            elif "r_m" in lowered:
                fmt = "maptool"
            continue

        values = [float(f) for f in fields if _is_number(f)]
        current.append(values)

    if current:
        blocks.append(current)
        block_poses.append(seen_pose)

    scans = []
    for k, rows in enumerate(blocks):
        rows_ok = [r for r in rows if len(r) >= 3]
        if len(rows_ok) < 10:
            continue                      # bloc trop court (scan partiel)
        name = path.stem if len(blocks) == 1 else f"{path.stem}_{k:03d}"

        if fmt == "esp32":
            # N, angle(°), distance(mm), intensity
            angle = [r[1] for r in rows_ok]
            dist = [r[2] / 1000.0 for r in rows_ok]
            inten = [r[3] if len(r) > 3 else 0 for r in rows_ok]
        elif fmt == "own":
            angle = [r[0] for r in rows_ok]
            dist = [r[1] / 1000.0 for r in rows_ok]
            inten = [r[2] for r in rows_ok]
        elif fmt == "robotcontrol":
            angle = [r[0] for r in rows_ok]
            dist = [r[1] for r in rows_ok]
            inten = [r[2] for r in rows_ok]
        else:  # maptool
            angle = [r[0] for r in rows_ok]
            dist = [r[1] for r in rows_ok]
            inten = [255 for _ in rows_ok]

        scan = _make_scan(name, angle, dist, inten)
        scan.pose = block_poses[k]
        scans.append(scan)
    return scans


def load_scan_folder(folder):
    """Charge tous les .csv d'un dossier (ordre alphabétique)."""
    folder = Path(folder)
    scans = []
    for file in sorted(folder.glob("*.csv")):
        scans.extend(load_scans(file))
    return scans


def save_scan(scan, path, pose=None):
    """Sauvegarde un scan au format maison : angle_deg,distance_mm,intensity.

    La position (`pose`, ou à défaut `scan.pose`) est écrite en commentaire sur la première ligne :
    le fichier garde ainsi la position du scan, et `load_scans` la retrouve.
    """
    path = Path(path)
    pose = pose if pose is not None else scan.pose
    angles_deg = np.degrees(scan.angles)
    lines = [pose_comment(pose)] if pose is not None else []
    lines.append("angle_deg,distance_mm,intensity")
    for a, r, i in zip(angles_deg, scan.ranges, scan.intensity):
        lines.append(f"{a:.3f},{r * 1000.0:.0f},{int(i)}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def load_poses(path):
    """Lit un fichier JSON {nom_du_scan: [x_m, y_m, theta_deg]}."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    poses = {name: tuple(float(v) for v in pose) for name, pose in data.items()}
    return poses


def save_poses(poses, path):
    """Écrit les poses dans un JSON lisible."""
    data = {name: [round(float(v), 4) for v in pose] for name, pose in poses.items()}
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n",
                          encoding="utf-8")


def poses_for(scans, poses=None):
    """{nom: pose} pour des scans : le dictionnaire `poses` (fichier poses.json) l'emporte, sinon la
    position écrite dans le .csv. Les scans sans position connue sont absents du résultat."""
    result = {}
    for scan in scans:
        if poses and scan.name in poses:
            result[scan.name] = tuple(poses[scan.name])
        elif scan.pose is not None:
            result[scan.name] = tuple(scan.pose)
    return result
