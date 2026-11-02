"""Tests de la partie sans interface de l'application : flux série, poses relatives, export."""

import json
import math
import sys
import zipfile
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lidar_io import load_poses, load_scans  # noqa: E402
from lidar_session import Session, compose_pose  # noqa: E402
from lidar_stream import (StreamParser, explain_serial_error, port_label,  # noqa: E402
                          rows_to_scan)
from make_synthetic import TRUE_POSES, simulate_scan  # noqa: E402

DATA = Path(__file__).resolve().parents[1] / "data_synth"


def csv_text(name):
    return (DATA / f"{name}.csv").read_text(encoding="utf-8")


def collect(text, chunk=137, **kw):
    scans = []
    parser = StreamParser(scans.append, **kw)
    for i in range(0, len(text), chunk):            # le texte arrive par morceaux arbitraires
        parser.feed(text[i:i + chunk])
    return scans


# ------------------------------------------------------------- décodage du flux
def test_parser_csv_reassembles_scans_from_arbitrary_chunks():
    text = csv_text("scan_000") + "\n" + csv_text("scan_001")
    scans = collect(text)
    assert len(scans) == 2
    assert all(len(rows) > 300 for rows in scans)


def test_parser_csv_without_blank_line_splits_on_angle_wrap():
    one = [l for l in csv_text("scan_000").splitlines() if l.strip()]
    text = "\n".join(one) + "\n" + "\n".join(one[1:]) + "\n"      # pas de ligne vide entre les tours
    scans = collect(text)
    assert len(scans) >= 1


def test_parser_ignores_headers_and_short_scans():
    text = "ets Jun 8 boot\nN,Angle(°),Distance(mm),Intensity,x(mm),y(mm)\n0,1.0,500,100,0,0\n\n"
    assert collect(text) == []


def test_parser_teleplot_matches_polar_data():
    angles = np.arange(10, 190, 2.0)               # demi-tour : le miroir se voit
    dist = 2000.0 + 0 * angles
    # la librairie imprime un y de signe opposé : x = r cos a, y = -r sin a
    pairs = ";".join(f"{d * math.cos(math.radians(a)):.1f}:{-d * math.sin(math.radians(a)):.1f}"
                     for a, d in zip(angles, dist))
    scans = collect(f">lidar:{pairs}|xy\n")
    assert len(scans) == 1
    got = np.array(scans[0])
    assert np.allclose(got[:, 0], angles, atol=0.1)
    assert np.allclose(got[:, 1], 2000.0, atol=0.5)
    flipped = collect(f">lidar:{pairs}|xy\n", tele_flip_y=False)
    assert np.allclose(np.array(flipped[0])[:, 0], (-angles) % 360, atol=0.1)


def test_rows_to_scan_offset_and_mirror():
    rows = [[a, 1000.0, 100.0] for a in range(0, 360, 5)]
    base = rows_to_scan(rows)
    shifted = rows_to_scan(rows, offset_deg=90)
    mirrored = rows_to_scan(rows, mirror=True)
    assert len(base) == 72
    assert set(np.round(np.degrees(shifted.angles)).astype(int) % 360) == {(a + 90) % 360 for a in range(0, 360, 5)}
    assert set(np.round(np.degrees(mirrored.angles)).astype(int) % 360) == {(-a) % 360 for a in range(0, 360, 5)}


# ------------------------------------------------------------- poses
def test_compose_pose_forward_then_turn():
    p = compose_pose((1.0, 1.5, 0.0), (1.0, 0.0, 90.0))
    assert np.allclose(p, (2.0, 1.5, 90.0))
    p = compose_pose(p, (1.0, 0.0, 0.0))                  # avance vers +y
    assert np.allclose(p, (2.0, 2.5, 90.0))
    p = compose_pose(p, (0.0, 0.5, 0.0))                  # 0,5 m à gauche = vers -x
    assert np.allclose(p, (1.5, 2.5, 90.0))


def test_compose_pose_wraps_angle():
    assert math.isclose(compose_pose((0, 0, 170), (0, 0, 30))[2], -160.0)


def test_relative_equals_absolute_chain():
    s = Session()
    rng = np.random.default_rng(0)
    scan = lambda pose: rows_to_scan([[a, r * 1000, 100] for a, r in zip(*simulate_scan(pose, rng)) if not np.isnan(r)])
    s.capture(scan((1.0, 1.5, 0.0)), "p", s.pose_from_input("abs", (1.0, 1.5, 0.0)))
    s.capture(scan((2.0, 1.5, 90.0)), "p", s.pose_from_input("rel", (1.0, 0.0, 90.0)))
    assert np.allclose(s.entries[1].pose, (2.0, 1.5, 90.0))
    assert [e.name for e in s.entries] == ["p_000", "p_001"]


def test_first_scan_defines_origin_even_in_relative_mode():
    assert Session().pose_from_input("rel", (0, 0, 0)) == (0.0, 0.0, 0.0)


def test_duplicate_names_are_made_unique():
    s = Session()
    scans = load_scans(DATA / "scan_000.csv")
    s.add_scans(scans)
    s.add_scans(scans)
    assert len({e.name for e in s.entries}) == 2


# ------------------------------------------------------------- cartes et export
def build_true_session():
    s = Session()
    poses = load_poses(DATA / "poses_true.json")
    for name in TRUE_POSES:
        s.add_scans(load_scans(DATA / f"{name}.csv"), poses)
    return s


def test_rebuild_gives_same_map_as_command_line_tools():
    s = build_true_session()
    s.rebuild()
    assert len(s.vmap.segments) == 11
    cls = s.grid.classify()
    assert int((cls == 1).sum()) == 453 and int((cls == 0).sum()) == 8993


def test_export_zip_content_and_roundtrip(tmp_path):
    s = build_true_session()
    s.rebuild()
    out = tmp_path / "relevé.zip"
    s.export_zip(out, png_bytes=b"\x89PNG fake")
    with zipfile.ZipFile(out) as zf:
        names = set(zf.namelist())
        assert {"poses.json", "projet.json", "carte_vectorielle.txt", "grille.csv", "carte.png"} <= names
        assert {f"scans/{n}.csv" for n in TRUE_POSES} <= names
        zf.extractall(tmp_path / "x")
    # le dossier exporté est relisible par les outils existants
    folder = tmp_path / "x"
    poses = load_poses(folder / "poses.json")
    assert set(poses) == set(TRUE_POSES)
    again = Session()
    assert again.import_files([folder / "projet.json"]) == 4
    again.rebuild()
    assert len(again.vmap.segments) == 11


def test_import_poses_json_updates_existing_scans(tmp_path):
    s = Session()
    s.add_scans(load_scans(DATA / "scan_000.csv"))
    (tmp_path / "p.json").write_text(json.dumps({"scan_000": [3, 2, 45]}))
    s.import_files([tmp_path / "p.json"])
    assert s.entries[0].pose == [3.0, 2.0, 45.0]


# ------------------------------------------------------------- erreurs série
def test_explain_serial_error():
    text, busy = explain_serial_error(PermissionError("could not open port 'COM5': PermissionError(13, 'Accès refusé.')"))
    assert busy and "occupé" in text
    text, busy = explain_serial_error(OSError("could not open port 'COM9': FileNotFoundError(2, 'x')"))
    assert not busy and "introuvable" in text
    text, _ = explain_serial_error(ValueError("autre"))
    assert "autre" in text


def test_port_label():
    assert port_label({"device": "COM5", "description": "Silicon Labs", "vendor": "CP210x (Silicon Labs)"}) \
        == "COM5 · CP210x (Silicon Labs)"
    assert port_label({"device": "COM3", "description": "n/a", "vendor": ""}) == "COM3"


def test_scan_colors_are_distinct_and_unlimited():
    from lidar_session import scan_color
    colors = [scan_color(i) for i in range(40)]
    assert len(set(colors)) == 40
    assert all(c.startswith("#") and len(c) == 7 for c in colors)


# ------------------------------------------------------------- vrai port série (pseudo-terminal)
@pytest.mark.skipif(not hasattr(__import__("os"), "openpty"), reason="pseudo-terminal indisponible (Windows)")
def test_serial_source_reads_scans_from_a_real_serial_device():
    import os
    import queue
    import threading
    import time

    pytest.importorskip("serial")
    from lidar_stream import SerialSource

    master, slave = os.openpty()
    events = queue.Queue()
    source = SerialSource(os.ttyname(slave), 115200, events)
    source.start()
    try:
        first = events.get(timeout=3)
        assert first[0] == "opened"

        def feed():
            data = (csv_text("scan_000") + "\n" + csv_text("scan_001")).encode("utf-8")
            for i in range(0, len(data), 300):
                os.write(master, data[i:i + 300])
                time.sleep(0.002)

        threading.Thread(target=feed, daemon=True).start()
        scans, deadline = [], time.time() + 5
        while len(scans) < 2 and time.time() < deadline:
            try:
                kind, payload = events.get(timeout=0.5)
            except queue.Empty:
                continue
            if kind == "scan":
                scans.append(payload)
        assert len(scans) == 2 and all(len(rows) > 300 for rows in scans)
        assert source.total_bytes > 10_000
    finally:
        source.stop()
        source.join(timeout=2)
        os.close(master)
        os.close(slave)
    assert not source.is_alive()


def test_serial_source_reports_unopenable_port():
    import queue

    pytest.importorskip("serial")
    from lidar_stream import SerialSource

    events = queue.Queue()
    source = SerialSource("/dev/ce_port_nexiste_pas" if sys.platform != "win32" else "COM250", 115200, events)
    source.start()
    kind, exc = events.get(timeout=3)
    assert kind == "open_error"
    text, _busy = explain_serial_error(exc)
    assert text
