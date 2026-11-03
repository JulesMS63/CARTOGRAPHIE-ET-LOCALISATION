"""Grille d'occupation 2D, sans filtre et avec filtre, en ligne de commande (point 3 du TP, version « filtrée »).

    python grid2d_tool.py data_synth --poses data_synth/poses_true.json --cell-mm 50 --out carte_grille2d
Sans --poses, les positions sont lues en tête des fichiers .csv (ligne « # pose: ... »).

Sorties :
    <out>_sans_filtre.png, <out>_avec_filtre.png : les deux grilles
    <out>_comparaison.png                        : les deux côte à côte + le tableau des cellules
    <out>_stats.txt                              : le tableau (cellules occupées, isolées, points ignorés)
    <out>_avec_filtre.csv                        : la grille filtrée (1 occupé, 0 libre, -1 inconnu)

Réglages du filtre : --min-points 8  --min-range 0.20  --break-min 0.10   (--no-filter : aucun filtre)
                     --confirm [--confirm-tol 0.10]  : ignorer aussi ce qu'un seul scan voit
Pour juger l'effet du filtre : --sweep-points 1,4,8,12 affiche le tableau pour plusieurs valeurs de --min-points.
"""

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import lidar_plot  # noqa: E402
from grid2d import clamp_cell_mm, stats_table  # noqa: E402
from lidar_io import load_poses, load_scan_folder, load_scans, poses_for  # noqa: E402
from lidar_session import MapParams, Session  # noqa: E402


def load_session(inputs, poses_path):
    scans = []
    for item in inputs:
        path = Path(item)
        scans.extend(load_scan_folder(path) if path.is_dir() else load_scans(path))
    if not scans:
        sys.exit("Aucun scan valide trouvé.")
    from_file = load_poses(poses_path) if poses_path else None
    poses = poses_for(scans, from_file)
    scans = [s for s in scans if s.name in poses]
    if not scans:
        sys.exit("Aucun scan n'a de pose (ni dans --poses, ni dans la ligne « # pose » des .csv).")
    session = Session()
    session.add_scans(scans, poses)
    return session


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("inputs", nargs="+", help="dossier(s) ou fichier(s) .csv")
    parser.add_argument("--poses", default=None)
    parser.add_argument("--cell-mm", type=int, default=50, help="taille des carreaux en mm, entier de 10 à 500")
    parser.add_argument("--forget", type=float, default=1.0)
    parser.add_argument("--min-points", type=int, default=MapParams.min_points)
    parser.add_argument("--min-range", type=float, default=MapParams.min_range)
    parser.add_argument("--break-min", type=float, default=MapParams.break_min)
    parser.add_argument("--confirm", action="store_true")
    parser.add_argument("--confirm-tol", type=float, default=MapParams.confirm_tol)
    parser.add_argument("--no-filter", action="store_true")
    parser.add_argument("--sweep-points", default=None, help="ex. 1,4,8,12")
    parser.add_argument("--out", default="carte_grille2d")
    args = parser.parse_args()

    session = load_session(args.inputs, args.poses)
    cell_mm = clamp_cell_mm(args.cell_mm)
    params = MapParams(resolution=cell_mm / 1000.0, forget=args.forget, clean=not args.no_filter,
                       min_points=args.min_points, min_range=args.min_range, break_min=args.break_min,
                       confirm=args.confirm, confirm_tol=args.confirm_tol)
    session.rebuild(params)
    pair = session.grid_pair(cell_mm, args.forget)
    if pair.capped:
        print(f"Zone très étendue : carreaux portés à {pair.cell_mm} mm.")

    print(f"{len(session.entries)} scan(s), carreaux de {pair.cell_mm} mm, grille {pair.raw.nx}×{pair.raw.ny}")
    table = stats_table(pair.stats)
    print(table)

    if args.sweep_points:
        print("\nEffet de --min-points :")
        print(f"{'points min.':>12s}{'occupées':>10s}{'isolées':>9s}{'ignorés':>9s}")
        for n in (int(v) for v in args.sweep_points.split(",")):
            session.rebuild(MapParams(**{**params.__dict__, "min_points": n}))
            st = session.grid_pair(cell_mm, args.forget).stats
            print(f"{n:12d}{st['flt']['occ']:10d}{st['flt']['iso']:9d}{st['ignored']:9d}")
        print(f"{'(sans filtre)':>12s}{pair.stats['raw']['occ']:10d}{pair.stats['raw']['iso']:9d}{0:9d}")
        session.rebuild(params)
        pair = session.grid_pair(cell_mm, args.forget)

    out = Path(args.out)
    for mode, suffix, title in (("raw", "sans_filtre", "Sans filtre"), ("flt", "avec_filtre", "Avec filtre")):
        fig, ax = plt.subplots(figsize=(8, 6), facecolor=lidar_plot.CANVAS)
        lidar_plot.draw_grid(ax, session, pair, mode)
        ax.set_title(f"Grille d'occupation 2D · {title} · carreaux de {pair.cell_mm} mm", color=lidar_plot.INK, fontsize=10)
        fig.savefig(f"{out}_{suffix}.png", dpi=130, bbox_inches="tight")
        plt.close(fig)

    Path(f"{out}_comparaison.png").write_bytes(lidar_plot.render_comparison_png(session, pair))

    Path(f"{out}_stats.txt").write_text(f"Carreaux : {pair.cell_mm} mm, {len(session.entries)} scan(s)\n"
                                        f"Filtre : {session.filter}\n\n{table}\n", encoding="utf-8")
    pair.flt.save_csv(f"{out}_avec_filtre.csv")
    print(f"-> {out}_sans_filtre.png, {out}_avec_filtre.png, {out}_comparaison.png, {out}_stats.txt, {out}_avec_filtre.csv")


if __name__ == "__main__":
    main()
