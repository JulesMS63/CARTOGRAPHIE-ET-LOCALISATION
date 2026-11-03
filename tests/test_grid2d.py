"""Tests de la grille d'occupation 2D « sans filtre / avec filtre » (grid2d.py, grid2d_tool.py, lidar_plot.draw_grid)."""

import subprocess
import sys
from pathlib import Path

import matplotlib
import numpy as np
import pytest

matplotlib.use("Agg")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import grid2d  # noqa: E402
import lidar_plot  # noqa: E402
from lidar_io import _make_scan  # noqa: E402
from lidar_session import MapParams, Session  # noqa: E402
from make_synthetic import TRUE_POSES, simulate_scan  # noqa: E402

DATA = ROOT / "data_synth"


def noisy_session(n_parasites=30, seed=3):
    """Quatre scans synthétiques ; chaque scan reçoit des mesures parasites isolées (une seule par angle)."""
    rng = np.random.default_rng(seed)
    session = Session()
    for pose in TRUE_POSES.values():
        angles, ranges = simulate_scan(pose, rng)
        ok = ~np.isnan(ranges)
        angles, ranges = np.asarray(angles)[ok], np.asarray(ranges)[ok].copy()
        ranges[rng.choice(len(ranges), n_parasites, replace=False)] = rng.uniform(0.3, 3.0, n_parasites)
        session.capture(_make_scan("s", angles, ranges, np.full(len(ranges), 100.0)), "scan", pose)
    return session


def test_clamp_cell_mm_is_an_integer_within_limits():
    assert grid2d.clamp_cell_mm("50") == 50
    assert grid2d.clamp_cell_mm("5") == grid2d.CELL_MM_MIN
    assert grid2d.clamp_cell_mm("100000") == grid2d.CELL_MM_MAX
    assert grid2d.clamp_cell_mm("") == 50
    assert grid2d.clamp_cell_mm("abc", default=40) == 40
    assert isinstance(grid2d.clamp_cell_mm(37.6), int)


def test_pair_uses_same_extent_and_filter_removes_isolated_cells():
    session = noisy_session()
    session.rebuild(MapParams(resolution=0.05))
    pair = session.grid_pair(50, 1.0)
    assert (pair.raw.nx, pair.raw.ny) == (pair.flt.nx, pair.flt.ny)
    assert pair.raw.extent() == pytest.approx(pair.flt.extent())
    st = pair.stats
    assert st["flt"]["iso"] < st["raw"]["iso"] / 2
    assert st["flt"]["occ"] >= 0.6 * st["raw"]["occ"]          # les murs restent
    assert st["ignored"] > 0 and st["total"] > st["ignored"]


def test_filter_off_gives_identical_grids():
    session = noisy_session()
    session.rebuild(MapParams(clean=False))
    pair = session.grid_pair(50, 1.0)
    assert pair.stats["raw"] == pair.stats["flt"]
    assert pair.stats["ignored"] == 0


def test_smaller_cells_make_a_finer_grid():
    session = noisy_session()
    session.rebuild(MapParams())
    coarse, fine = session.grid_pair(100, 1.0), session.grid_pair(25, 1.0)
    assert fine.raw.nx > 3 * coarse.raw.nx
    assert fine.cell_mm == 25 and not fine.capped


def test_huge_area_enlarges_the_cells_instead_of_exploding(monkeypatch):
    session = noisy_session()
    session.rebuild(MapParams())
    monkeypatch.setattr(grid2d, "MAX_CELLS", 5000)
    pair = grid2d.build_grid_pair(session.visible(), session.used, 10, 1.0)
    assert pair.capped and pair.cell_mm > 10
    assert pair.raw.nx * pair.raw.ny < 20000


def test_no_scan_gives_no_pair():
    assert Session().grid_pair(50, 1.0) is None


def test_stats_table_mentions_both_columns():
    session = noisy_session()
    session.rebuild(MapParams())
    table = grid2d.stats_table(session.grid_pair(50, 1.0).stats)
    assert "Sans filtre" in table and "Avec filtre" in table and "isolées" in table


def test_draw_grid_modes_and_wipe_move():
    from matplotlib.figure import Figure
    session = noisy_session()
    session.rebuild(MapParams())
    pair = session.grid_pair(50, 1.0)
    fig = Figure()
    ax = fig.add_subplot(111)
    for mode in ("raw", "flt", "cmp"):
        art = lidar_plot.draw_grid(ax, session, pair, mode, wipe=0.25)
    xl = ax.get_xlim()
    assert lidar_plot.wipe_fraction(art) == pytest.approx(0.25, abs=1e-6)
    lidar_plot.move_wipe(art, 0.8)
    assert lidar_plot.wipe_fraction(art) == pytest.approx(0.8, abs=1e-6)
    ax.set_xlim(xl[0] + 1, xl[1] - 1)                          # zoom : la fraction se mesure sur la vue actuelle
    lidar_plot.move_wipe(art, 0.5)
    assert art["line"].get_xdata()[0] == pytest.approx(sum(ax.get_xlim()) / 2)
    assert lidar_plot.draw_grid(ax, session, None) == {}


def test_render_comparison_png_is_a_png():
    session = noisy_session()
    session.rebuild(MapParams())
    data = lidar_plot.render_comparison_png(session, session.grid_pair(50, 1.0))
    assert data[:8] == b"\x89PNG\r\n\x1a\n" and len(data) > 5000


def test_cli_writes_images_stats_and_sweep(tmp_path):
    out = tmp_path / "g"
    run = subprocess.run([sys.executable, str(ROOT / "grid2d_tool.py"), str(DATA), "--poses", str(DATA / "poses_true.json"), "--cell-mm", "40",
                          "--sweep-points", "1,8", "--out", str(out)], capture_output=True, text=True, cwd=ROOT)
    assert run.returncode == 0, run.stderr
    for suffix in ("_sans_filtre.png", "_avec_filtre.png", "_comparaison.png", "_stats.txt", "_avec_filtre.csv"):
        assert Path(f"{out}{suffix}").exists(), suffix
    assert "Effet de --min-points" in run.stdout and "40 mm" in run.stdout
    assert "Cellules occupées" in Path(f"{out}_stats.txt").read_text(encoding="utf-8")
