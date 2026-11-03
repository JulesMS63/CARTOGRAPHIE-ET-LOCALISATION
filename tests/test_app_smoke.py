"""Test de fumée de l'application de bureau : démarre la fenêtre, simule un lidar, capture, exporte.

Ignoré automatiquement s'il n'y a pas d'écran (serveur, intégration continue) ou pas de Tkinter.
"""

import sys
import time
import zipfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

tk = pytest.importorskip("tkinter")


@pytest.fixture
def app():
    import lidar_app
    try:
        instance = lidar_app.App()
    except tk.TclError as exc:
        pytest.skip(f"pas d'écran disponible : {exc}")
    errors = []
    instance.report_callback_exception = lambda *a: errors.append(a)
    yield instance
    instance.stop_source()
    instance.destroy()
    assert not errors, errors


def pump(app, seconds):
    end = time.time() + seconds
    while time.time() < end:
        app.update()
        time.sleep(0.01)


def test_simulation_capture_map_export(app, tmp_path, monkeypatch):
    import lidar_app
    assert app.steps["connect"].state == "current"

    app.toggle_sim()
    pump(app, 1.2)
    assert app.live is not None and app.steps["capture"].state == "current"

    for pose in ((1.0, 1.5, 0.0), (2.5, 3.0, 35.0)):
        for var, value in zip(app.cap_vars, pose):
            var.set(str(value))
        pump(app, 0.5)
        app.capture_now()
    assert len(app.session.entries) == 2
    assert app.steps["capture"].state == "done"

    app.pose_mode.set("rel")
    app.on_mode_change()
    app.cap_vars[0].set("1")
    app.capture_now()
    assert len(app.session.entries) == 3
    x, y, th = app.session.entries[2].pose
    assert abs(x - (2.5 + 0.8191520442889918)) < 1e-6      # 1 m vers l'avant à 35 degrés

    app.set_tab("map")
    pump(app, 0.5)
    assert app.session.vmap is not None and app.session.vmap.segments

    app.set_tab("grid")                                       # étape 5 : grille 2D sans / avec filtre
    pump(app, 0.5)
    assert app.steps["grid"].state == "done" and app.last_pair is not None
    assert {"line", "left", "right"} <= set(app.grid_art)

    target = tmp_path / "releve.zip"
    monkeypatch.setattr(lidar_app.filedialog, "asksaveasfilename", lambda **kw: str(target))
    app.export_zip()
    with zipfile.ZipFile(target) as zf:
        names = set(zf.namelist())
    assert {"poses.json", "projet.json", "carte_vectorielle.txt", "grille.csv", "carte.png"} <= names
    assert {"grille2d_comparaison.png", "grille2d_stats.txt"} <= names     # ajoutés car la grille 2D a été ouverte
    assert app.steps["export"].state == "done"


def test_grid_tab_controls(app):
    import lidar_app
    from make_synthetic import TRUE_POSES, simulate_scan
    import numpy as np
    from lidar_io import _make_scan
    rng = np.random.default_rng(1)
    for pose in list(TRUE_POSES.values())[:2]:
        a, r = simulate_scan(pose, rng)
        ok = ~np.isnan(r)
        app.session.capture(_make_scan("s", np.asarray(a)[ok], np.asarray(r)[ok], np.full(int(ok.sum()), 100.0)),
                            "scan", pose)
    app.render_scan_list()
    assert app.steps["grid"].state != "done"                  # pas encore consultée
    app.set_tab("grid")
    pump(app, 0.4)
    assert app.steps["grid"].state == "done"

    # taille des carreaux : entier en mm, ramené à 10-500
    app.res_var.set("5")
    app.on_cell_change()
    assert app.res_var.get() == "10"
    app.res_var.set("20")
    app.on_cell_change()
    pump(app, 0.3)
    assert app.last_pair.cell_mm == 20
    assert app.params().resolution == pytest.approx(0.02)
    assert lidar_app.digits_only("12") and lidar_app.digits_only("") and not lidar_app.digits_only("1.5")

    # curseur « force du filtre » : 0 désactive le filtre, sinon il règle « Points min. »
    app.strength.set(0)
    pump(app, 0.5)
    assert not app.clean_var.get() and app.last_pair.stats["flt"] == app.last_pair.stats["raw"]
    app.strength.set(11)
    pump(app, 0.5)
    assert app.clean_var.get() and app.minpts_var.get() == "11"
    app.minpts_var.set("6")
    app.on_param_change()
    assert app.strength.get() == 6                            # le curseur suit le champ

    # séparation : reste à la fraction demandée après un zoom
    app.wipe.set(30)
    app.apply_wipe()
    import types
    app.on_scroll(types.SimpleNamespace(inaxes=app.ax, button="up", xdata=3.0, ydata=2.0))
    xl, xw = app.ax.get_xlim(), app.grid_art["line"].get_xdata()[0]
    assert (xw - xl[0]) / (xl[1] - xl[0]) == pytest.approx(0.3, abs=1e-6)

    app.session.clear()
    app.flags.update(grid_seen=False)
    app.last_pair = None
    app.redraw()
    app.update_chrome()


def test_connection_without_port_explains_what_to_check(app, monkeypatch):
    import lidar_app
    monkeypatch.setattr(lidar_app, "list_ports", lambda: [])
    monkeypatch.setattr(lidar_app, "pyserial_available", lambda: True)
    monkeypatch.setattr(app, "open_diagnostic", lambda: None)
    app.port_var.set("")
    app.connect()
    assert "Aucun port" in app.conn_status.cget("text")
    assert app.source is None
