"""Filtre des points isolés : cas simples, bords, et comparaison avec la version JavaScript."""

import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from lidar_io import Scan, _make_scan  # noqa: E402
from lidar_session import MapParams, Session  # noqa: E402
from make_synthetic import TRUE_POSES, simulate_scan  # noqa: E402
from scanfilter import filter_isolated, isolated_mask  # noqa: E402


def wall_scan(n=360, dist=3.0):
    """Un tour complet : cercle parfait de rayon `dist` (un point par degré)."""
    return _make_scan("mur", np.arange(n) * 360.0 / n, np.full(n, dist), np.full(n, 100.0))


def with_values(scan, replace):
    """Copie du scan où les points d'indices donnés prennent de nouvelles distances."""
    r = scan.ranges.copy()
    for i, v in replace.items():
        r[i] = v
    return Scan(scan.name, scan.angles.copy(), r, scan.intensity.copy())


def test_clean_scan_is_untouched():
    s = wall_scan()
    assert isolated_mask(s).all()
    assert len(filter_isolated(s)) == len(s)


def test_single_outlier_is_removed():
    s = with_values(wall_scan(), {100: 0.5})
    keep = isolated_mask(s)
    assert not keep[100] and keep.sum() == len(s) - 1


def test_outlier_inside_wall_does_not_cut_the_wall():
    """Un parasite au milieu d'un petit morceau de mur ne doit pas le couper en deux et le faire disparaître."""
    s = wall_scan(n=360)
    # morceau de 12 points à 3 m, le reste à 6 m (donc coupé de ce morceau) ; un parasite en plein milieu
    r = np.full(len(s), 6.0)
    r[40:52] = 3.0
    r[46] = 0.4
    keep = isolated_mask(Scan("x", s.angles, r, s.intensity))
    assert keep[40:46].all() and keep[47:52].all() and not keep[46]


def test_small_group_is_removed_big_group_kept():
    r = np.full(360, 6.0)
    r[10:13] = 1.0            # 3 points : trop petit
    r[200:210] = 1.0          # 10 points : conservé
    s = Scan("x", np.radians(np.arange(360.0)), r, np.full(360, 100.0))
    keep = isolated_mask(s)
    assert not keep[10:13].any() and keep[200:210].all()
    assert isolated_mask(s, min_points=3)[10:13].all()           # réglable


def test_too_close_points_are_removed():
    s = with_values(wall_scan(), {i: 0.05 for i in range(20, 40)})      # 20 points voisins, mais collés au capteur
    keep = isolated_mask(s)
    assert not keep[20:40].any()
    assert isolated_mask(s, min_range=0.0)[20:40].all()


def test_wraparound_group_is_one_group():
    """Un obstacle à cheval sur 0°/360° est un seul groupe, pas deux demi-groupes de 3 points."""
    r = np.full(360, 6.0)
    ring = [355, 356, 357, 358, 359, 0, 1, 2, 3, 4]
    r[ring] = 1.0
    s = Scan("x", np.radians(np.arange(360.0)), r, np.full(360, 100.0))
    assert isolated_mask(s)[ring].all()


def test_degenerate_scans():
    empty = Scan("e", np.array([]), np.array([]), np.array([]))
    assert len(filter_isolated(empty)) == 0
    one = Scan("u", np.array([0.1]), np.array([2.0]), np.array([1.0]))
    assert len(filter_isolated(one)) == 0
    assert len(filter_isolated(one, min_points=1)) == 1
    two = Scan("d", np.array([0.1, 0.2]), np.array([2.0, 2.0]), np.array([1.0, 1.0]))
    assert len(filter_isolated(two)) == 0
    assert len(filter_isolated(two, min_points=2)) == 2


def test_input_scan_not_modified():
    s = with_values(wall_scan(), {100: 0.5})
    before = s.ranges.copy()
    filter_isolated(s)
    assert np.array_equal(s.ranges, before) and len(s) == 360


def noisy_room_scan(seed):
    rng = np.random.default_rng(seed)
    a, r = simulate_scan(TRUE_POSES["scan_000"] if "scan_000" in TRUE_POSES else list(TRUE_POSES.values())[0], rng)
    ok = ~np.isnan(r)
    a, r = np.asarray(a)[ok], np.asarray(r)[ok]
    kind = np.zeros(len(a), dtype=int)                       # 0 vrai, 1 parasite
    r = r.copy()
    idx = rng.choice(len(a), 30, replace=False)
    r[idx[:20]] = rng.uniform(0.2, 5.0, 20)                  # isolés, un seul retour par angle
    r[idx[20:]] = rng.uniform(0.03, 0.09, 10)                # collés au capteur
    kind[idx] = 1
    s = _make_scan("bruit", a, r, np.full(len(a), 100.0))
    order = np.argsort(np.mod(a, 360.0))
    return s, kind[order]


def test_noisy_room_most_noise_removed_few_true_points_lost():
    lost = true = noise = noise_left = 0
    for seed in range(10):
        s, kind = noisy_room_scan(seed)
        keep = isolated_mask(s)
        true += (kind == 0).sum()
        lost += ((kind == 0) & ~keep).sum()
        noise += (kind == 1).sum()
        noise_left += ((kind == 1) & keep).sum()
    assert lost / true < 0.03                    # moins de 3 % des vrais points perdus
    assert noise_left / noise < 0.25             # la plupart des parasites partent (certains tombent par hasard sur un mur)


def test_synthetic_clean_scans_barely_change():
    for seed in range(3):
        rng = np.random.default_rng(seed)
        a, r = simulate_scan(list(TRUE_POSES.values())[0], rng)
        ok = ~np.isnan(r)
        s = _make_scan("c", np.asarray(a)[ok], np.asarray(r)[ok], np.full(int(ok.sum()), 100.0))
        assert isolated_mask(s).mean() > 0.95


# ---- dans la session --------------------------------------------------------
def make_session(clean):
    sess = Session()
    s, _ = noisy_room_scan(3)
    sess.capture(s, "t", (0.0, 0.0, 0.0))
    sess.rebuild(MapParams(clean=clean))
    return sess, s


def test_session_filters_but_keeps_raw():
    sess, raw = make_session(True)
    e = sess.entries[0]
    assert len(e.scan) == len(raw)                              # brut intact
    assert len(e.used(sess.filter)) < len(raw)
    assert "ignoré" in sess.map_info
    raw_data = sess.project_dict()["scans"][0]["distances_mm"]
    assert len(raw_data) == len(raw)                            # projet.json brut


def test_session_filter_off_uses_raw():
    sess, raw = make_session(False)
    assert len(sess.entries[0].used(sess.filter)) == len(raw)
    assert "ignoré" not in sess.map_info


def test_filtering_changes_the_map_for_the_better():
    on, _ = make_session(True)
    off, _ = make_session(False)
    assert len(on.vmap.segments) <= len(off.vmap.segments)         # moins de petits segments parasites


def test_export_csv_filtered_project_raw(tmp_path):
    sess, raw = make_session(True)
    sess.export_folder(tmp_path)
    csv_lines = (tmp_path / "scans" / f"{sess.entries[0].name}.csv").read_text().strip().splitlines()
    assert csv_lines[0].startswith("# pose:") and len(csv_lines) - 2 == len(sess.used(sess.entries[0]))
    proj = json.loads((tmp_path / "projet.json").read_text())
    assert len(proj["scans"][0]["distances_mm"]) == len(raw)


def test_filter_params_cache_follows_changes():
    sess, _ = make_session(True)
    e = sess.entries[0]
    n1 = len(e.used(sess.filter))
    sess.rebuild(MapParams(clean=True, min_points=12))
    assert len(e.used(sess.filter)) <= n1
    sess.rebuild(MapParams(clean=True, min_points=1, min_range=0.0))
    assert len(e.used(sess.filter)) == len(e.scan)


# ---- parité avec le JavaScript ----------------------------------------------
@pytest.mark.skipif(shutil.which("node") is None, reason="node absent")
def test_same_result_as_javascript(tmp_path):
    html = ROOT / "lidar_web.html"
    if not html.exists():
        html = ROOT.parent / "lidar_web.html"
    if not html.exists():
        pytest.skip("lidar_web.html introuvable")
    scans = [noisy_room_scan(seed)[0] for seed in range(6)] + [wall_scan(), with_values(wall_scan(), {10: 0.4, 11: 0.4})]
    cases = [{"ang_deg": np.degrees(s.angles).tolist(), "rng": s.ranges.tolist()} for s in scans]
    (tmp_path / "cases.json").write_text(json.dumps(cases))
    (tmp_path / "run.js").write_text(r"""
const fs = require('fs');
const html = fs.readFileSync(process.argv[2], 'utf8');
const core = html.split('/* CORE-START */')[1].split('/* CORE-END */')[0];
const api = new Function(core + '; return {makeScan, filterScan};')();
const cases = JSON.parse(fs.readFileSync(process.argv[3]));
console.log(JSON.stringify(cases.map(c => {
  const s = api.makeScan('x', c.ang_deg, c.rng, c.ang_deg.map(() => 100));
  return Array.from(api.filterScan(s, {minPoints: 8, minRange: 0.2, breakMin: 0.1}).ranges, v => +v.toFixed(9));
})));
""")
    out = subprocess.check_output(["node", str(tmp_path / "run.js"), str(html), str(tmp_path / "cases.json")])
    got = json.loads(out)
    for s, g in zip(scans, got):
        assert np.round(filter_isolated(s).ranges, 9).tolist() == g


# ---- confirmation par un autre scan ----------------------------------------
def room_scans(seed=0, noise=0):
    """4 scans de la salle simulée, avec `noise` parasites isolés par scan ; retourne (scans, poses)."""
    rng = np.random.default_rng(seed)
    scans, poses = [], {}
    for k, pose in enumerate(list(TRUE_POSES.values())):
        a, r = simulate_scan(pose, rng)
        ok = ~np.isnan(r)
        a, r = np.asarray(a)[ok], np.asarray(r)[ok].copy()
        if noise:
            idx = rng.choice(len(a), noise, replace=False)
            r[idx] = rng.uniform(0.3, 5.0, noise)
        s = _make_scan(f"s{k}", a, r, np.full(len(a), 100.0))
        scans.append(s)
        poses[s.name] = pose
    return scans, poses


def test_confirm_removes_points_seen_once():
    from scanfilter import confirm_scans
    scans, poses = room_scans(1, noise=25)
    done = confirm_scans(scans, poses, 0.10)
    assert all(len(d) <= len(s) for d, s in zip(done, scans))
    removed = sum(len(s) - len(d) for s, d in zip(scans, done))
    assert removed > 40                                 # les parasites partent (et un peu de zones vues une seule fois)
    assert len(confirm_scans(scans[:1], poses)[0]) == len(scans[0])        # un seul scan : rien n'est retiré


def test_confirm_keeps_walls():
    from scanfilter import confirm_scans
    scans, poses = room_scans(2)
    done = confirm_scans(scans, poses, 0.10)
    kept = sum(len(d) for d in done) / sum(len(s) for s in scans)
    assert kept > 0.75                                  # la salle est vue par plusieurs scans : les murs restent


@pytest.mark.skipif(shutil.which("node") is None, reason="node absent")
def test_confirm_same_result_as_javascript(tmp_path):
    from scanfilter import confirm_scans
    html = ROOT / "lidar_web.html"
    if not html.exists():
        pytest.skip("lidar_web.html introuvable")
    scans, poses = room_scans(3, noise=20)
    data = [{"ang_deg": np.degrees(s.angles).tolist(), "rng": s.ranges.tolist(), "pose": list(poses[s.name])} for s in scans]
    (tmp_path / "cases.json").write_text(json.dumps(data))
    (tmp_path / "run.js").write_text(r"""
const fs = require('fs');
const html = fs.readFileSync(process.argv[2], 'utf8');
const core = html.split('/* CORE-START */')[1].split('/* CORE-END */')[0];
const api = new Function(core + '; return {makeScan, confirmScans};')();
const cases = JSON.parse(fs.readFileSync(process.argv[3]));
const scans = cases.map((c, i) => { const s = api.makeScan('s' + i, c.ang_deg, c.rng, c.ang_deg.map(() => 100)); s.pose = {x: c.pose[0], y: c.pose[1], th: c.pose[2]}; return s; });
console.log(JSON.stringify(api.confirmScans(scans, 0.10).map(s => Array.from(s.ranges, v => +v.toFixed(9)))));
""")
    got = json.loads(subprocess.check_output(["node", str(tmp_path / "run.js"), str(html), str(tmp_path / "cases.json")]))
    for s, g in zip(confirm_scans(scans, poses, 0.10), got):
        assert np.round(s.ranges, 9).tolist() == g


# ---- position dans le fichier .csv ----------------------------------------
def test_pose_follows_the_csv_file(tmp_path):
    from lidar_io import load_scans, save_scan
    s = wall_scan()
    save_scan(s, tmp_path / "a.csv", pose=(1.23456, -0.5, -37.25))
    first = (tmp_path / "a.csv").read_text().splitlines()[0]
    assert first == "# pose: x=1.2346 y=-0.5000 theta=-37.2500"
    back = load_scans(tmp_path / "a.csv")[0]
    assert back.pose == (1.2346, -0.5, -37.25) and len(back) == len(s)
    save_scan(s, tmp_path / "b.csv")
    assert load_scans(tmp_path / "b.csv")[0].pose is None            # pas de position connue : pas de ligne « # pose »


def test_pose_exported_by_the_web_page_is_read(tmp_path):
    from lidar_io import load_scans
    text = "# pose: x=3.2500 y=-1.5000 theta=42.0000\nangle_deg,distance_mm,intensity\n" + "\n".join(
        f"{a}.000,2000,90" for a in range(0, 360, 5)) + "\n"
    (tmp_path / "scan_001.csv").write_text(text)
    assert load_scans(tmp_path / "scan_001.csv")[0].pose == (3.25, -1.5, 42.0)


def test_session_import_and_export_keep_poses(tmp_path):
    sess = Session()
    for k, pose in enumerate([(1.0, 2.0, 10.0), (3.0, 1.0, -90.0)]):
        sess.capture(wall_scan(dist=3.0 + k), "s", pose)
    sess.rebuild(MapParams(clean=False))
    sess.export_folder(tmp_path)
    files = sorted((tmp_path / "scans").glob("*.csv"))
    other = Session()
    other.import_files(files)                                      # seulement les .csv, pas de poses.json
    assert [tuple(e.pose) for e in other.entries] == [(1.0, 2.0, 10.0), (3.0, 1.0, -90.0)]
    assert other.last_import_posed == 2


def test_tools_use_pose_written_in_csv(tmp_path):
    """gridmap_tool build sans --poses : les positions viennent des .csv."""
    from lidar_io import save_scan
    scans, poses = room_scans(4)
    for s in scans:
        save_scan(s, tmp_path / f"{s.name}.csv", pose=poses[s.name])
    out = tmp_path / "carte"
    subprocess.check_call([sys.executable, str(ROOT / "gridmap_tool.py"), "build", str(tmp_path), "--out", str(out)],
                          stdout=subprocess.DEVNULL, cwd=str(ROOT))
    assert out.with_suffix(".npz").exists() and out.with_suffix(".png").exists()
