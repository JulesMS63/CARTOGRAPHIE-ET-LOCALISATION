"""Tests de bout en bout : lecture de fichiers, grille d'occupation, carte vectorielle."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry import transform_points  # noqa: E402
from gridmap import FREE, OCCUPIED, UNKNOWN, OccupancyGrid, grid_from_scan, ray_cells  # noqa: E402
from lidar_io import Scan, load_scans, save_scan  # noqa: E402
from make_synthetic import TRUE_POSES, simulate_scan, write_esp32_csv  # noqa: E402
from vectormap import VectorMap  # noqa: E402


def square_room_scan(half=2.0, n=360):
    """Scan parfait d'une pièce carrée de côté 2*half, lidar au centre."""
    angles = np.radians(np.arange(n) * 360.0 / n)
    ranges = []
    for a in angles:
        c, s = abs(np.cos(a)), abs(np.sin(a))
        ranges.append(half / max(c, s))
    return Scan("room", angles, np.array(ranges), np.full(n, 100.0))


# ------------------------------------------------------------------ lecture
def test_roundtrip_own_format(tmp_path):
    scan = square_room_scan()
    path = tmp_path / "s.csv"
    save_scan(scan, path)
    loaded = load_scans(path)[0]
    assert len(loaded) == len(scan)
    assert np.allclose(loaded.ranges, scan.ranges, atol=1e-3)


def test_esp32_format_two_scans_in_one_file(tmp_path):
    rng = np.random.default_rng(0)
    path = tmp_path / "log.csv"
    angles, ranges = simulate_scan(TRUE_POSES["scan_000"], rng)
    block = ["N,Angle(°),Distance(mm),Intensity,x(mm),y(mm)"]
    for blk in range(2):
        for n, (a, r) in enumerate(zip(angles, ranges)):
            if not np.isnan(r):
                block.append(f"{n},{a:.2f},{int(r * 1000)},100,0,0")
        block.append("")
    path.write_text("\n".join(block), encoding="utf-8")
    scans = load_scans(path)
    assert len(scans) == 2


def test_invalid_ranges_are_dropped(tmp_path):
    path = tmp_path / "s.csv"
    rows = ["angle_deg,distance_mm,intensity"]
    for k in range(50):
        rows.append(f"{k * 7.0},{0 if k % 2 else 2000},100")
    path.write_text("\n".join(rows), encoding="utf-8")
    scan = load_scans(path)[0]
    assert len(scan) == 25


def test_ld06_angle_convention_wall_in_front():
    """Un obstacle à 1 m en face (angle math 0°) doit être en x = +1, y = 0."""
    scan = Scan("t", np.array([0.0]), np.array([1.0]), np.array([100.0]))
    assert np.allclose(scan.points_local(), [[1.0, 0.0]])


# ------------------------------------------------------------------ pose
def test_transform_points_rotation_and_translation():
    pts = np.array([[1.0, 0.0]])
    out = transform_points(pts, (2.0, 3.0, 90.0))
    assert np.allclose(out, [[2.0, 4.0]])


# ------------------------------------------------------------------ grille
def test_ray_cells_horizontal_excludes_last():
    ii, jj = ray_cells(0, 0, 4, 0)
    assert list(ii) == [0, 1, 2, 3]
    assert list(jj) == [0, 0, 0, 0]


def test_ray_cells_same_cell_is_empty():
    ii, _ = ray_cells(2, 2, 2, 2)
    assert len(ii) == 0


def test_grid_from_scan_square_room():
    scan = square_room_scan(half=2.0)
    grid = grid_from_scan(scan, (0.0, 0.0, 0.0), resolution=0.05, margin=0.5)
    state = grid.classify()
    assert (state == OCCUPIED).sum() > 100
    ci, cj = grid.world_to_cell(0.0, 0.0)
    assert state[cj, ci] == FREE                         # le centre est libre
    wi, wj = grid.world_to_cell(2.0, 0.0)
    assert state[wj, wi] == OCCUPIED or state[wj, wi - 1] == OCCUPIED   # mur à x = 2
    ui, uj = grid.world_to_cell(2.4, 0.0)
    assert state[uj, ui] == UNKNOWN                      # derrière le mur : inconnu


def test_grid_update_moves_probability_toward_certainty():
    scan = square_room_scan(half=2.0)
    grid = grid_from_scan(scan, (0.0, 0.0, 0.0), resolution=0.1)
    before = grid.log_odds.copy()
    grid.update(scan, (0.0, 0.0, 0.0))
    assert grid.log_odds.max() > before.max() - 1e-12
    assert grid.log_odds.min() <= before.min() + 1e-12
    assert grid.log_odds.max() <= grid.l_max and grid.log_odds.min() >= grid.l_min


def test_grid_hit_beats_grazing_free():
    """Une cellule d'impact traversée par un autre rayon du même scan reste occupée."""
    scan = square_room_scan(half=2.0)
    grid = grid_from_scan(scan, (0.0, 0.0, 0.0), resolution=0.05)
    prob = grid.probability()
    i, j = grid.world_to_cell(0.0, 2.0)
    assert prob[j - 1:j + 2, i].max() > 0.5


def test_grid_update_outside_bounds_does_not_crash():
    scan = square_room_scan(half=2.0)
    grid = OccupancyGrid(-1.0, -1.0, 1.0, 1.0, resolution=0.1)   # trop petite
    grid.update(scan, (0.0, 0.0, 0.0))
    grid.update(scan, (5.0, 5.0, 0.0))                           # lidar hors de la carte


def test_grid_forgetting_decays():
    scan = square_room_scan(half=1.5)
    grid = grid_from_scan(scan, (0, 0, 0), resolution=0.1, forget=0.5)
    max_after_first = grid.log_odds.max()
    empty = Scan("e", np.zeros(0), np.zeros(0), np.zeros(0))
    grid.update(empty, (0, 0, 0))
    assert grid.log_odds.max() == pytest.approx(max_after_first * 0.5)


def test_grid_npz_roundtrip(tmp_path):
    scan = square_room_scan()
    grid = grid_from_scan(scan, (0, 0, 0))
    grid.save_npz(tmp_path / "g.npz")
    other = OccupancyGrid.load_npz(tmp_path / "g.npz")
    assert np.allclose(other.log_odds, grid.log_odds)
    assert other.nx == grid.nx and other.ny == grid.ny


# ------------------------------------------------------------------ carte vectorielle
def test_vector_map_square_room_gives_four_walls():
    scan = square_room_scan(half=2.0, n=450)
    vmap = VectorMap()
    vmap.update(scan, (0.0, 0.0, 0.0))
    assert len(vmap.segments) == 4
    for s in vmap.segments:
        assert s.length == pytest.approx(4.0, abs=0.25)


def test_vector_map_same_scan_twice_merges():
    scan = square_room_scan(half=2.0, n=450)
    vmap = VectorMap()
    vmap.update(scan, (0.0, 0.0, 0.0))
    vmap.update(scan, (0.0, 0.0, 0.0))
    assert len(vmap.segments) == 4
    assert all(s.weight >= 2.0 for s in vmap.segments)


def test_vector_map_synthetic_multi_pose():
    rng = np.random.default_rng(1)
    vmap = VectorMap()
    for name, pose in TRUE_POSES.items():
        angles, ranges = simulate_scan(pose, rng)
        ok = ~np.isnan(ranges)
        scan = Scan(name, np.radians(angles[ok]), ranges[ok], np.full(ok.sum(), 100.0))
        vmap.update(scan, pose)
    # la salle (4 murs) + pilier (4) + meuble (3) : on s'attend à un nombre proche de 11
    assert 8 <= len(vmap.segments) <= 20
    longest = max(s.length for s in vmap.segments)
    assert longest > 3.5


def test_vector_map_save_load(tmp_path):
    scan = square_room_scan(half=2.0, n=450)
    vmap = VectorMap()
    vmap.update(scan, (0.0, 0.0, 0.0))
    vmap.save(tmp_path / "m.txt")
    loaded = VectorMap.load(tmp_path / "m.txt")
    assert len(loaded) == len(vmap.segments)


# ------------------------------------------------------------------ formats des outils du module
def test_maptool_format_zero_range_means_no_hit(tmp_path):
    rows = ["angle_deg,r_m,x_rel_m,y_rel_m"]
    for k in range(40):
        a = k * 9.0
        r = 0 if k % 4 == 0 else 2.0
        rows.append(f"{a:.3f},{r:.4f},0.0,0.0")
    path = tmp_path / "scan_lidar.csv"
    path.write_text("\n".join(rows), encoding="utf-8")
    scan = load_scans(path)[0]
    assert len(scan) == 30                      # les 10 points r_m = 0 sont ignorés
    assert np.allclose(scan.ranges, 2.0)


def test_robotcontrol_format_distance_in_meters(tmp_path):
    rows = ["# pose: 0,0,0", "angle_deg,distance_m,intensity"]
    for k in range(30):
        rows.append(f"{k * 12.0:.2f},1.500,90")
    path = tmp_path / "lidar_scan.csv"
    path.write_text("\n".join(rows), encoding="utf-8")
    scan = load_scans(path)[0]
    assert len(scan) == 30 and np.allclose(scan.ranges, 1.5)
