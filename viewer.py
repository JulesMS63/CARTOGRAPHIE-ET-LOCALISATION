"""Visualiseur de nuages de points lidar (plusieurs scans, une couleur par scan).

Lancer :
    python viewer.py data_synth                    # dossier de .csv
    python viewer.py data_synth --poses poses.json # poses de départ
    python viewer.py scan_a.csv scan_b.csv         # fichiers

Commandes :
    - boutons radio (à gauche) : choisir le scan actif
    - curseurs x, y, theta : pose du scan actif
    - clavier : flèches = déplace de 5 cm (Maj : 1 cm) | , et . = tourne de 1° (Maj : 0.2°)
                Tab = scan suivant | s = sauver les poses | r = remettre la pose du scan à 0
Les poses sont sauvegardées dans un JSON {nom_du_scan: [x_m, y_m, theta_deg]}.
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, RadioButtons, Slider

from lidar_io import load_poses, load_scan_folder, load_scans, poses_for, save_poses

COLORS = ["tab:blue", "tab:red", "tab:green", "tab:orange", "tab:purple",
          "tab:brown", "tab:pink", "tab:olive", "tab:cyan", "tab:gray"]


class ScanViewer:
    def __init__(self, scans, poses, poses_path):
        self.scans = scans
        self.poses = {s.name: tuple(poses.get(s.name, (0.0, 0.0, 0.0))) for s in scans}
        self.poses_path = poses_path
        self.active = 0
        self._updating = False

        self.fig = plt.figure(figsize=(12, 7))
        self.ax = self.fig.add_axes([0.25, 0.22, 0.72, 0.74])
        self.ax.set_aspect("equal")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")

        self.clouds = []
        self.origins = []
        for k, scan in enumerate(self.scans):
            color = COLORS[k % len(COLORS)]
            cloud = self.ax.scatter([], [], s=6, color=color, label=scan.name)
            (origin,) = self.ax.plot([], [], marker="o", color=color, ms=9,
                                     markeredgecolor="k")
            (heading,) = self.ax.plot([], [], "-", color=color, lw=2)
            self.clouds.append(cloud)
            self.origins.append((origin, heading))
        self.ax.legend(loc="upper right", fontsize=8)

        # liste des scans
        ax_radio = self.fig.add_axes([0.02, 0.45, 0.19, 0.5])
        labels = [s.name for s in self.scans]
        self.radio = RadioButtons(ax_radio, labels)
        for k, text in enumerate(self.radio.labels):
            text.set_color(COLORS[k % len(COLORS)])
            text.set_fontsize(9)
        self.radio.on_clicked(self.on_select)

        # curseurs de pose
        ax_x = self.fig.add_axes([0.30, 0.12, 0.55, 0.03])
        ax_y = self.fig.add_axes([0.30, 0.07, 0.55, 0.03])
        ax_t = self.fig.add_axes([0.30, 0.02, 0.55, 0.03])
        self.slider_x = Slider(ax_x, "x (m)", -10.0, 10.0, valinit=0.0, valstep=0.01)
        self.slider_y = Slider(ax_y, "y (m)", -10.0, 10.0, valinit=0.0, valstep=0.01)
        self.slider_t = Slider(ax_t, "theta (°)", -180.0, 180.0, valinit=0.0, valstep=0.1)
        for slider in (self.slider_x, self.slider_y, self.slider_t):
            slider.on_changed(self.on_slider)

        # boutons
        ax_save = self.fig.add_axes([0.02, 0.30, 0.19, 0.06])
        ax_reset = self.fig.add_axes([0.02, 0.22, 0.19, 0.06])
        self.button_save = Button(ax_save, "Sauver poses (s)")
        self.button_reset = Button(ax_reset, "Pose scan = 0 (r)")
        self.button_save.on_clicked(lambda _e: self.save())
        self.button_reset.on_clicked(lambda _e: self.reset_active())

        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

        self.refresh_all()
        self.sync_sliders()
        self.fit_view()

    # ---- affichage ------------------------------------------------------
    def refresh_scan(self, k):
        scan = self.scans[k]
        pose = self.poses[scan.name]
        points = scan.points_world(pose)
        self.clouds[k].set_offsets(points if len(points) else np.zeros((0, 2)))
        x, y, theta = pose
        origin, heading = self.origins[k]
        origin.set_data([x], [y])
        arrow = 0.3
        tx = x + arrow * np.cos(np.radians(theta))
        ty = y + arrow * np.sin(np.radians(theta))
        heading.set_data([x, tx], [y, ty])

    def refresh_all(self):
        for k in range(len(self.scans)):
            self.refresh_scan(k)
        self.fig.canvas.draw_idle()

    def fit_view(self):
        all_pts = [s.points_world(self.poses[s.name]) for s in self.scans if len(s)]
        if not all_pts:
            return
        pts = np.vstack(all_pts)
        margin = 0.5
        self.ax.set_xlim(pts[:, 0].min() - margin, pts[:, 0].max() + margin)
        self.ax.set_ylim(pts[:, 1].min() - margin, pts[:, 1].max() + margin)

    # ---- interactions ---------------------------------------------------
    def sync_sliders(self):
        """Met les curseurs à la pose du scan actif (sans déclencher on_slider)."""
        x, y, theta = self.poses[self.scans[self.active].name]
        self._updating = True
        self.slider_x.set_val(x)
        self.slider_y.set_val(y)
        self.slider_t.set_val(theta)
        self._updating = False

    def on_select(self, label):
        names = [s.name for s in self.scans]
        self.active = names.index(label)
        self.sync_sliders()

    def on_slider(self, _value):
        if self._updating:
            return
        name = self.scans[self.active].name
        self.poses[name] = (self.slider_x.val, self.slider_y.val, self.slider_t.val)
        self.refresh_scan(self.active)
        self.fig.canvas.draw_idle()

    def set_active_pose(self, x, y, theta):
        name = self.scans[self.active].name
        theta = (theta + 180.0) % 360.0 - 180.0
        self.poses[name] = (x, y, theta)
        self.sync_sliders()
        self.refresh_scan(self.active)
        self.fig.canvas.draw_idle()

    def on_key(self, event):
        key = event.key
        if key is None:
            return
        x, y, theta = self.poses[self.scans[self.active].name]
        step = 0.01 if "shift" in key else 0.05
        rot = 0.2 if "shift" in key else 1.0
        base = key.replace("shift+", "")
        if base == "left":
            self.set_active_pose(x - step, y, theta)
        elif base == "right":
            self.set_active_pose(x + step, y, theta)
        elif base == "up":
            self.set_active_pose(x, y + step, theta)
        elif base == "down":
            self.set_active_pose(x, y - step, theta)
        elif base in (",", "<"):
            self.set_active_pose(x, y, theta + rot)
        elif base in (".", ">"):
            self.set_active_pose(x, y, theta - rot)
        elif base == "tab":
            self.active = (self.active + 1) % len(self.scans)
            self.radio.set_active(self.active)
        elif base == "s":
            self.save()
        elif base == "r":
            self.reset_active()

    def reset_active(self):
        self.set_active_pose(0.0, 0.0, 0.0)

    def save(self):
        save_poses(self.poses, self.poses_path)
        print(f"Poses sauvegardées dans {self.poses_path}")


def collect_scans(inputs):
    scans = []
    for item in inputs:
        path = Path(item)
        if path.is_dir():
            scans.extend(load_scan_folder(path))
        else:
            scans.extend(load_scans(path))
    return scans


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("inputs", nargs="+", help="dossier(s) ou fichier(s) .csv")
    parser.add_argument("--poses", default="poses.json",
                        help="fichier de poses à lire / écrire (défaut : poses.json)")
    args = parser.parse_args()

    scans = collect_scans(args.inputs)
    if not scans:
        sys.exit("Aucun scan valide trouvé.")
    print(f"{len(scans)} scan(s) chargé(s) : {[s.name for s in scans]}")

    from_file = None
    if Path(args.poses).exists():
        from_file = load_poses(args.poses)
        print(f"Poses lues dans {args.poses}")
    poses = poses_for(scans, from_file)          # sinon : position écrite en tête de chaque .csv
    if len(poses) and from_file is None:
        print(f"Positions relues dans les fichiers .csv pour {len(poses)} scan(s)")

    viewer = ScanViewer(scans, poses, args.poses)
    plt.show()
    return viewer


if __name__ == "__main__":
    main()
