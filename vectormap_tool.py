"""Construit une carte vectorielle à partir de scans + poses (point 5 du TP).

    python vectormap_tool.py data_synth --poses poses_true.json --out vecmap
(sans --poses : positions lues en tête des .csv, ligne « # pose: ... »)
Sorties : <out>.txt (x1,y1,x2,y2 en m, format du module) et <out>.png

Paramètres modifiables : --ang-tol (°), --dist-tol (m), --gap-tol (m), --min-weight,
--no-cut (désactive la résolution des croisements)
"""

import argparse
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from lidar_io import load_poses, load_scan_folder, poses_for  # noqa: E402
from vectormap import VectorMap, plot_segments  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder")
    parser.add_argument("--poses", default=None, help="poses.json ; défaut : poses écrites dans les .csv")
    parser.add_argument("--out", default="vecmap")
    parser.add_argument("--ang-tol", type=float, default=10.0)
    parser.add_argument("--dist-tol", type=float, default=0.06)
    parser.add_argument("--gap-tol", type=float, default=0.15)
    parser.add_argument("--min-weight", type=float, default=1.0,
                        help="poids minimal pour garder un segment (1 = on garde tout)")
    parser.add_argument("--no-cut", action="store_true")
    args = parser.parse_args()

    scans = load_scan_folder(args.folder)
    poses = poses_for(scans, load_poses(args.poses) if args.poses else None)
    scans = [s for s in scans if s.name in poses]
    if not scans:
        sys.exit("Aucun scan n'a de pose (ni dans --poses, ni dans la ligne « # pose » des .csv).")

    vmap = VectorMap(ang_tol_deg=args.ang_tol, dist_tol=args.dist_tol,
                     gap_tol=args.gap_tol, cut_crossings=not args.no_cut)
    fig, ax = plt.subplots(figsize=(8, 6))
    for scan in scans:
        pose = poses[scan.name]
        pts = scan.points_world(pose)
        ax.scatter(pts[:, 0], pts[:, 1], s=2, color="lightgray")
        new = vmap.update(scan, pose)
        print(f"{scan.name}: {len(new)} segments extraits, carte = {len(vmap.segments)} segments")

    vmap.prune(min_weight=args.min_weight)
    plot_segments(ax, vmap.segments, color="tab:blue")
    ax.set_title(f"Carte vectorielle : {len(vmap.segments)} segments")
    ax.grid(True, alpha=0.3)
    fig.savefig(f"{args.out}.png", dpi=130, bbox_inches="tight")
    vmap.save(f"{args.out}.txt")
    print(f"-> {args.out}.txt et {args.out}.png")


if __name__ == "__main__":
    main()
