"""Tests des opérations sur segments, dont les cas limites exigés par le TP.

Lancer depuis le dossier du projet :   python -m pytest -v
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from segments import (EPS_LEN, Segment, angle_between, are_close, cut_by,  # noqa: E402
                      extract_segments, intersection, merge, segment_distance)


def seg(x1, y1, x2, y2):
    return Segment((x1, y1), (x2, y2))


def same(a, b, tol=1e-9):
    return (np.allclose(a.p1, b.p1, atol=tol) and np.allclose(a.p2, b.p2, atol=tol))


# ------------------------------------------------------------------ angle
def test_angle_perpendicular():
    assert angle_between(seg(0, 0, 1, 0), seg(0, 0, 0, 1)) == pytest.approx(math.pi / 2)


def test_angle_oriented_vs_undirected():
    a = seg(0, 0, 1, 0)
    b = seg(1, 0, 0, 0)                         # même droite, sens opposé
    assert angle_between(a, b, oriented=True) == pytest.approx(math.pi)
    assert angle_between(a, b, oriented=False) == pytest.approx(0.0, abs=1e-12)


def test_angle_45():
    assert angle_between(seg(0, 0, 1, 0), seg(0, 0, 1, 1)) == pytest.approx(math.pi / 4)


# ------------------------------------------------------------------ proximité
def test_close_collinear_with_small_gap():
    a = seg(0, 0, 1, 0)
    b = seg(1.05, 0.0, 2, 0.0)
    assert are_close(a, b, gap_tol=0.10)
    assert not are_close(a, b, gap_tol=0.01)


def test_not_close_when_angle_too_large():
    a = seg(0, 0, 1, 0)
    b = seg(1, 0, 2, 0.5)                       # ~26.6°
    assert not are_close(a, b, ang_tol=math.radians(10))


def test_not_close_when_parallel_but_far():
    a = seg(0, 0, 1, 0)
    b = seg(0, 0.5, 1, 0.5)
    assert not are_close(a, b, dist_tol=0.05)


def test_close_is_symmetric():
    a = seg(0, 0, 2, 0)
    b = seg(1.5, 0.03, 2.4, 0.04)
    assert are_close(a, b) == are_close(b, a)


# ------------------------------------------------------------------ fusion
def test_merge_collinear_covers_both():
    merged = merge(seg(0, 0, 1, 0), seg(1.05, 0, 2, 0))
    assert merged.length == pytest.approx(2.0)
    assert merged.p1[0] == pytest.approx(0.0)
    assert merged.p2[0] == pytest.approx(2.0)


def test_merge_keeps_direction_of_first():
    merged = merge(seg(2, 0, 0, 0), seg(1, 0.0, -1, 0.0))
    assert merged.p2[0] < merged.p1[0]          # toujours orienté vers -x


def test_merge_offset_parallel_gives_middle_line():
    merged = merge(seg(0, 0.00, 1, 0.00), seg(0, 0.02, 1, 0.02))
    assert merged.p1[1] == pytest.approx(0.01)
    assert merged.p2[1] == pytest.approx(0.01)


# ------------------------------------------------------------------ intersection
def test_intersection_cross():
    point, t, u = intersection(seg(0, 0, 2, 0), seg(1, -1, 1, 1))
    assert np.allclose(point, (1, 0))
    assert t == pytest.approx(0.5) and u == pytest.approx(0.5)


def test_intersection_none_when_lines_cross_outside():
    assert intersection(seg(0, 0, 1, 0), seg(2, -1, 2, 1)) is None


def test_intersection_none_when_parallel():
    assert intersection(seg(0, 0, 1, 0), seg(0, 1, 1, 1)) is None


def test_intersection_none_when_collinear():
    assert intersection(seg(0, 0, 2, 0), seg(1, 0, 3, 0)) is None


def test_segment_distance_zero_when_crossing():
    assert segment_distance(seg(0, 0, 2, 0), seg(1, -1, 1, 1)) == 0.0


# ------------------------------------------------------------------ découpe
def test_cut_keeps_longer_side_first():
    a = seg(0, 0, 4, 0)                          # croisé à x = 3 -> côtés 3 et 1
    b = seg(3, -1, 3, 1)
    assert same(cut_by(a, b), seg(0, 0, 3, 0))


def test_cut_keeps_longer_side_second():
    a = seg(0, 0, 4, 0)                          # croisé à x = 1 -> côtés 1 et 3
    b = seg(1, -1, 1, 1)
    assert same(cut_by(a, b), seg(1, 0, 4, 0))


def test_cut_endpoint_of_seg_unchanged():
    a = seg(0, 0, 4, 0)
    b = seg(4, -1, 4, 1)                         # coupe EXACTEMENT à l'extrémité p2
    assert cut_by(a, b) is a
    c = seg(0, -1, 0, 1)                         # et à l'extrémité p1
    assert cut_by(a, c) is a


def test_cut_within_tol_of_endpoint_unchanged():
    tol = 1e-3
    a = seg(0, 0, 4, 0)
    b = seg(4 - 0.5 * tol, -1, 4 - 0.5 * tol, 1)   # à 0.5 tol du bout : "à l'extrémité"
    assert cut_by(a, b, tol=tol) is a
    c = seg(4 - 2 * tol, -1, 4 - 2 * tol, 1)       # à 2 tol : vrai découpage
    assert not same(cut_by(a, c, tol=tol), a, tol=1e-12)


def test_cut_collinear_not_cut():
    a = seg(0, 0, 4, 0)
    b = seg(1, 0, 2, 0)                          # colinéaire, recouvrement
    assert cut_by(a, b) is a
    c = seg(2, 0, 6, 0)
    assert cut_by(a, c) is a


def test_cut_parallel_not_cut():
    a = seg(0, 0, 4, 0)
    assert cut_by(a, seg(0, 1, 4, 1)) is a


def test_cut_no_intersection_unchanged():
    a = seg(0, 0, 4, 0)
    assert cut_by(a, seg(2, 1, 2, 2)) is a       # le cutter ne touche pas a


def test_cut_cutter_ends_on_seg_counts():
    a = seg(0, 0, 4, 0)
    b = seg(1, 0, 1, 2)                          # cutter qui s'arrête SUR a (jonction en T)
    assert same(cut_by(a, b), seg(1, 0, 4, 0))


def test_cut_equal_halves_keeps_first_point_side():
    a = seg(0, 0, 4, 0)
    b = seg(2, -1, 2, 1)
    assert same(cut_by(a, b), seg(0, 0, 2, 0))


def test_cut_equal_halves_reversed_segment():
    a = seg(4, 0, 0, 0)                          # p1 = (4, 0) : on garde le côté de (4, 0)
    b = seg(2, -1, 2, 1)
    assert same(cut_by(a, b), seg(4, 0, 2, 0))


def test_cut_tie_within_tol():
    tol = 1e-3
    a = seg(0, 0, 4, 0)
    b = seg(2 + 0.4 * tol, -1, 2 + 0.4 * tol, 1)   # côtés 2.0004 et 1.9996 : écart 0.8 tol
    assert same(cut_by(a, b, tol=tol), seg(0, 0, 2 + 0.4 * tol, 0), tol=1e-12)
    # si le segment est inversé, on garde toujours le côté de p1
    a_rev = seg(4, 0, 0, 0)
    kept = cut_by(a_rev, b, tol=tol)
    assert np.allclose(kept.p1, (4, 0))


def test_cut_just_above_tie_tolerance_keeps_longer():
    tol = 1e-3
    a = seg(0, 0, 4, 0)
    b = seg(2 - 2 * tol, -1, 2 - 2 * tol, 1)       # côté p1 = 1.998 < côté p2 = 2.002
    kept = cut_by(a, b, tol=tol)
    assert np.allclose(kept.p1, (2 - 2 * tol, 0)) and np.allclose(kept.p2, (4, 0))


def test_cut_cutter_just_misses_within_tol_still_counts():
    tol = 1e-3
    a = seg(0, 0, 4, 0)
    b = seg(1, 0.5 * tol, 1, 2)                   # s'arrête 0.5 tol AU-DESSUS de a
    assert same(cut_by(a, b, tol=tol), seg(1, 0, 4, 0), tol=1e-12)
    c = seg(1, 2 * tol, 1, 2)                     # s'arrête 2 tol au-dessus : pas de contact
    assert cut_by(a, c, tol=tol) is a


def test_cut_degenerate_segment_unchanged():
    a = seg(1, 1, 1, 1)
    assert cut_by(a, seg(0, 1, 2, 1)) is a


def test_cut_default_tolerance_is_documented():
    assert EPS_LEN == 1e-9


# ------------------------------------------------------------------ extraction
def make_wall_points(p, q, n):
    t = np.linspace(0, 1, n)[:, None]
    return np.array(p) + t * (np.array(q) - np.array(p))


def test_extract_two_walls_forming_corner():
    # un coin à angle droit, vu depuis (0, 0), points ordonnés par angle
    wall1 = make_wall_points((1.0, -1.0), (1.0, 1.0), 40)       # x = 1
    wall2 = make_wall_points((1.0, 1.0), (-1.0, 1.0), 40)[1:]   # y = 1
    points = np.vstack((wall1, wall2))
    segs = extract_segments(points)
    assert len(segs) == 2
    lengths = sorted(s.length for s in segs)
    assert lengths[0] == pytest.approx(2.0, abs=0.1)
    assert lengths[1] == pytest.approx(2.0, abs=0.1)


def test_extract_breaks_at_large_gap():
    wall1 = make_wall_points((1.0, -1.0), (1.0, -0.2), 30)
    wall2 = make_wall_points((1.0, 0.6), (1.0, 1.4), 30)       # trou de 0.8 m
    segs = extract_segments(np.vstack((wall1, wall2)))
    assert len(segs) == 2


def test_extract_ignores_tiny_clusters():
    pts = make_wall_points((1.0, 0.0), (1.0, 0.05), 4)
    assert extract_segments(pts) == []
