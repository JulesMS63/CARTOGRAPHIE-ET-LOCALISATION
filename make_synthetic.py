"""Génère des scans lidar SIMULÉS d'une salle, pour tester les outils sans robot.

Les fichiers produits imitent exactement la sortie ESP32 `printScanCSV` :
    N,Angle(°),Distance(mm),Intensity,x(mm),y(mm)
(les colonnes x et y sont volontairement mises à 0 : on ne les utilise pas).

Utilisation :
    python make_synthetic.py            -> crée data_synth/scan_000.csv ... + poses_true.json
"""

import argparse
from pathlib import Path

import numpy as np

from geometry import rotation_matrix
from lidar_io import save_poses

# Salle 6 m x 4 m + un pilier carré + un petit meuble contre le mur du bas.
ROOM_WALLS = [
    ((0.0, 0.0), (6.0, 0.0)),
    ((6.0, 0.0), (6.0, 4.0)),
    ((6.0, 4.0), (0.0, 4.0)),
    ((0.0, 4.0), (0.0, 0.0)),
    # pilier 0.5 x 0.5 m
    ((3.0, 2.0), (3.5, 2.0)),
    ((3.5, 2.0), (3.5, 2.5)),
    ((3.5, 2.5), (3.0, 2.5)),
    ((3.0, 2.5), (3.0, 2.0)),
    # meuble : un mur de 1.2 m en saillie
    ((1.0, 0.0), (1.0, 0.6)),
    ((1.0, 0.6), (2.2, 0.6)),
    ((2.2, 0.6), (2.2, 0.0)),
]

TRUE_POSES = {
    "scan_000": (1.0, 1.5, 0.0),
    "scan_001": (2.5, 3.0, 35.0),
    "scan_002": (4.8, 1.2, 120.0),
    "scan_003": (5.0, 3.2, -150.0),
}


def simulate_scan(pose, rng, n_points=450, noise_m=0.01, dropout=0.02, max_range=12.0):
    """Lance n_points rayons depuis `pose` et renvoie (angles_deg, distances_m)."""
    x, y, theta_deg = pose
    angles_local = np.linspace(0.0, 360.0, n_points, endpoint=False)
    origin = np.array([x, y])
    ranges = np.full(n_points, np.nan)

    for k, a_local in enumerate(angles_local):
        a_world = np.radians(a_local + theta_deg)
        direction = np.array([np.cos(a_world), np.sin(a_world)])
        best = np.inf
        for (p, q) in ROOM_WALLS:
            p = np.array(p)
            q = np.array(q)
            edge = q - p
            denom = direction[0] * edge[1] - direction[1] * edge[0]
            if abs(denom) < 1e-12:
                continue
            w = p - origin
            t = (w[0] * edge[1] - w[1] * edge[0]) / denom      # distance le long du rayon
            u = (w[0] * direction[1] - w[1] * direction[0]) / denom
            if t > 1e-6 and 0.0 <= u <= 1.0:
                best = min(best, t)
        if np.isfinite(best) and best <= max_range:
            ranges[k] = best + rng.normal(0.0, noise_m)

    drop = rng.random(n_points) < dropout
    ranges[drop] = np.nan
    return angles_local, ranges


def write_esp32_csv(path, angles_deg, ranges_m):
    lines = ["N,Angle(°),Distance(mm),Intensity,x(mm),y(mm)"]
    n = 0
    for a, r in zip(angles_deg, ranges_m):
        if np.isnan(r):
            continue
        lines.append(f"{n},{a:.2f},{int(round(r * 1000))},{120},0,0")
        n += 1
    lines.append("")
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="data_synth")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    for name, pose in TRUE_POSES.items():
        angles, ranges = simulate_scan(pose, rng)
        write_esp32_csv(out / f"{name}.csv", angles, ranges)
        print(f"{name}: pose vraie = {pose}")

    save_poses(TRUE_POSES, out / "poses_true.json")
    print(f"-> {out}/ (poses vraies dans poses_true.json)")


if __name__ == "__main__":
    main()
