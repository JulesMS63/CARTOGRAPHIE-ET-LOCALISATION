"""Outils en ligne de commande pour la carte d'occupation (point 3 du TP).

1) scan + pose  ->  carte d'occupation
       python gridmap_tool.py scan2grid scan.csv --pose 1.0 1.5 0 --out map1
2) carte + scan + pose  ->  carte mise à jour
       python gridmap_tool.py update map1.npz scan2.csv --pose 2.5 3.0 35 --out map2
3) tous les scans d'un dossier + poses.json  ->  carte finale (+ images étape par étape)
       python gridmap_tool.py build data_synth --poses poses_true.json --out map_all --steps
   Sans --poses, la position écrite en tête de chaque .csv (ligne « # pose: ... ») est utilisée.

Options utiles : --res 0.05 (taille de cellule en m), --forget 0.98 (oubli progressif)
Sorties : <out>.npz (carte complète), <out>.png (image), <out>.csv (1/0/-1)
"""

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from gridmap import OccupancyGrid, grid_for_scans, grid_from_scan, plot_grid  # noqa: E402
from lidar_io import load_poses, load_scan_folder, load_scans, poses_for  # noqa: E402


def save_outputs(grid, out, poses_list=None, title=""):
    out = Path(out)
    grid.save_npz(out.with_suffix(".npz"))
    grid.save_csv(out.with_suffix(".csv"))
    fig, ax = plt.subplots(figsize=(8, 6))
    plot_grid(grid, ax, poses_list)
    ax.set_title(title or f"Carte d'occupation ({grid.n_updates} scan(s), {grid.resolution} m)")
    fig.savefig(out.with_suffix(".png"), dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"-> {out.with_suffix('.npz')}, .csv, .png")


def first_scan(path):
    scans = load_scans(path)
    if not scans:
        sys.exit(f"Aucun scan valide dans {path}")
    return scans[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--res", type=float, default=0.05)
    common.add_argument("--forget", type=float, default=1.0)
    common.add_argument("--max-range", type=float, default=None)
    common.add_argument("--out", default="map")

    p1 = sub.add_parser("scan2grid", parents=[common])
    p1.add_argument("scan")
    p1.add_argument("--pose", type=float, nargs=3, default=None,
                    metavar=("X", "Y", "THETA_DEG"), help="défaut : pose écrite dans le .csv, sinon 0 0 0")

    p2 = sub.add_parser("update", parents=[common])
    p2.add_argument("grid", help="carte existante (.npz)")
    p2.add_argument("scan")
    p2.add_argument("--pose", type=float, nargs=3, default=None,
                    metavar=("X", "Y", "THETA_DEG"), help="défaut : pose écrite dans le .csv, sinon 0 0 0")

    p3 = sub.add_parser("build", parents=[common])
    p3.add_argument("folder")
    p3.add_argument("--poses", default=None, help="poses.json ; défaut : poses écrites dans les .csv")
    p3.add_argument("--steps", action="store_true", help="sauve une image après chaque scan")

    args = parser.parse_args()

    if args.command == "scan2grid":
        scan = first_scan(args.scan)
        pose = tuple(args.pose) if args.pose else (scan.pose or (0.0, 0.0, 0.0))
        grid = grid_from_scan(scan, pose, resolution=args.res, forget=args.forget)
        save_outputs(grid, args.out, [pose])

    elif args.command == "update":
        grid = OccupancyGrid.load_npz(args.grid)
        scan = first_scan(args.scan)
        pose = tuple(args.pose) if args.pose else (scan.pose or (0.0, 0.0, 0.0))
        n_free, n_occ = grid.update(scan, pose, max_range=args.max_range)
        print(f"{n_free} cellules libres et {n_occ} cellules occupées mises à jour")
        save_outputs(grid, args.out, [pose])

    else:  # build
        scans = load_scan_folder(args.folder)
        poses = poses_for(scans, load_poses(args.poses) if args.poses else None)
        scans = [s for s in scans if s.name in poses]
        if not scans:
            sys.exit("Aucun scan n'a de pose (ni dans --poses, ni dans la ligne « # pose » des .csv).")
        grid = grid_for_scans(scans, poses, resolution=args.res, forget=args.forget)
        used = []
        for k, scan in enumerate(scans):
            grid.update(scan, poses[scan.name], max_range=args.max_range)
            used.append(poses[scan.name])
            print(f"scan {k + 1}/{len(scans)} ({scan.name}) intégré")
            if args.steps:
                save_outputs(grid, f"{args.out}_step{k + 1}", used,
                             title=f"Après {k + 1} scan(s)")
        save_outputs(grid, args.out, used)


if __name__ == "__main__":
    main()
