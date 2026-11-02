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

    target = tmp_path / "releve.zip"
    monkeypatch.setattr(lidar_app.filedialog, "asksaveasfilename", lambda **kw: str(target))
    app.export_zip()
    with zipfile.ZipFile(target) as zf:
        names = set(zf.namelist())
    assert {"poses.json", "projet.json", "carte_vectorielle.txt", "grille.csv", "carte.png"} <= names
    assert app.steps["export"].state == "done"


def test_connection_without_port_explains_what_to_check(app, monkeypatch):
    import lidar_app
    monkeypatch.setattr(lidar_app, "list_ports", lambda: [])
    monkeypatch.setattr(lidar_app, "pyserial_available", lambda: True)
    monkeypatch.setattr(app, "open_diagnostic", lambda: None)
    app.port_var.set("")
    app.connect()
    assert "Aucun port" in app.conn_status.cget("text")
    assert app.source is None
