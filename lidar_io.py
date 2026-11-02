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

Dans tous les cas on recalcule x et y NOUS-MÊMES à partir de (angle, distance),
on n'utilise pas les colonnes x/y de la librairie (voir RAPPORT : signe de y).
"""

import json
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
    current = []

    for raw in lines:
        line = raw.strip()
        if line == "":
            if current:
                blocks.append(current)
                current = []
            continue
        if line.startswith("#"):
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

        scans.append(_make_scan(name, angle, dist, inten))
    return scans


def load_scan_folder(folder):
    """Charge tous les .csv d'un dossier (ordre alphabétique)."""
    folder = Path(folder)
    scans = []
    for file in sorted(folder.glob("*.csv")):
        scans.extend(load_scans(file))
    return scans


def save_scan(scan, path):
    """Sauvegarde un scan au format maison : angle_deg,distance_mm,intensity."""
    path = Path(path)
    angles_deg = np.degrees(scan.angles)
    lines = ["angle_deg,distance_mm,intensity"]
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
