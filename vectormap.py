"""Carte vectorielle 2D : un ensemble de segments dans le repère monde.

Pour chaque nouveau scan (pris depuis une pose connue) :
  1. extraction des segments dans le repère du lidar        (segments.extract_segments)
  2. passage dans le repère monde grâce à la pose
  3. pour chaque nouveau segment :
       - s'il est "proche" d'un segment de la carte -> fusion  (are_close + merge)
       - sinon -> ajout à la carte
  4. consolidation : les segments de la carte devenus proches entre eux sont fusionnés
  5. (option) résolution des croisements : si un nouveau segment CROISE franchement
     un ancien (intersection loin des deux extrémités ET angle >= 30°), l'ancien est
     découpé et on garde son plus long côté (cut_by)
  6. (option) filtrage : on retire les segments vus trop peu de fois (weight < min_weight)
"""

import math

import numpy as np

from geometry import transform_points
from segments import (Segment, angle_between, are_close, cut_by, extract_segments,
                      intersection, merge)


def segment_to_world(seg, pose):
    """Applique la pose (x, y, theta) aux deux extrémités d'un segment."""
    pts = transform_points(np.array([seg.p1, seg.p2]), pose)
    moved = Segment(pts[0], pts[1], weight=seg.weight)
    return moved


class VectorMap:
    def __init__(self, ang_tol_deg=10.0, dist_tol=0.06, gap_tol=0.15,
                 adaptive_angle=True, cut_crossings=True, cross_margin=0.05,
                 min_cross_angle_deg=30.0, **extract_kwargs):
        self.segments = []
        self.ang_tol = math.radians(ang_tol_deg)
        self.dist_tol = dist_tol
        self.gap_tol = gap_tol
        self.adaptive_angle = adaptive_angle
        self.cut_crossings = cut_crossings
        self.cross_margin = cross_margin
        self.min_cross_angle = math.radians(min_cross_angle_deg)
        self.extract_kwargs = extract_kwargs
        self.n_scans = 0

    def _is_close(self, a, b):
        """are_close avec un seuil angulaire qui s'élargit pour les segments courts.

        Sur un segment court, un bruit de quelques cm suffit à faire pivoter la droite
        ajustée de plusieurs degrés. Le critère d'écart latéral (dist_tol) garde la
        fusion cohérente géométriquement, donc on tolère un angle un peu plus grand :
            ang_tol_effectif = ang_tol + atan(dist_tol / longueur_du_plus_court)
        """
        ang_tol = self.ang_tol
        if self.adaptive_angle:
            shortest = max(min(a.length, b.length), 1e-9)
            ang_tol = self.ang_tol + math.atan2(self.dist_tol, shortest)
            ang_tol = min(ang_tol, math.radians(45.0))
        return are_close(a, b, ang_tol=ang_tol, dist_tol=self.dist_tol, gap_tol=self.gap_tol)

    def update(self, scan, pose):
        """Intègre un scan dans la carte. Retourne les nouveaux segments (monde)."""
        local_points = scan.points_local()
        local_segments = extract_segments(local_points, **self.extract_kwargs)
        new_segments = [segment_to_world(s, pose) for s in local_segments]
        self.add_segments(new_segments)
        self.n_scans += 1
        return new_segments

    def add_segments(self, new_segments):
        for new in new_segments:
            if self.cut_crossings:
                self._cut_old_segments_crossed_by(new)

            fused = False
            for k, old in enumerate(self.segments):
                if self._is_close(old, new):
                    self.segments[k] = merge(old, new)
                    fused = True
                    break
            if not fused:
                self.segments.append(new)

        # les fusions peuvent avoir rapproché des segments entre eux
        self._consolidate()

    def _consolidate(self):
        """Fusionne les segments de la carte devenus proches entre eux (jusqu'à stabilité)."""
        changed = True
        while changed:
            changed = False
            for i in range(len(self.segments)):
                for j in range(i + 1, len(self.segments)):
                    if self._is_close(self.segments[i], self.segments[j]):
                        self.segments[i] = merge(self.segments[i], self.segments[j])
                        del self.segments[j]
                        changed = True
                        break
                if changed:
                    break

    def _cut_old_segments_crossed_by(self, new):
        """Découpe les anciens segments franchement traversés par `new`."""
        for k, old in enumerate(self.segments):
            hit = intersection(old, new)
            if hit is None:
                continue
            _point, t, u = hit
            if angle_between(old, new) < self.min_cross_angle:
                continue            # quasi parallèles : bruit, pas un vrai croisement
            margin_old = self.cross_margin / max(old.length, 1e-9)
            margin_new = self.cross_margin / max(new.length, 1e-9)
            interior_old = margin_old < t < 1.0 - margin_old
            interior_new = margin_new < u < 1.0 - margin_new
            if interior_old and interior_new:        # vrai croisement en "X"
                self.segments[k] = cut_by(old, new)

    def prune(self, min_weight=2.0, min_length=0.15):
        """Filtre : retire les segments peu observés ou trop courts."""
        kept = [s for s in self.segments
                if s.weight >= min_weight and s.length >= min_length]
        self.segments = kept

    # ---- entrées / sorties ------------------------------------------------
    def save(self, path):
        """Format du module : une ligne `x1,y1,x2,y2` par segment, en mètres."""
        lines = ["# x1,y1,x2,y2 (m)"]
        for s in self.segments:
            x1, y1, x2, y2 = s.as_tuple()
            lines.append(f"{x1:.4f},{y1:.4f},{x2:.4f},{y2:.4f}")
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

    @staticmethod
    def load(path):
        segs = []
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                x1, y1, x2, y2 = (float(v) for v in line.split(","))
                segs.append(Segment((x1, y1), (x2, y2)))
        return segs


def plot_segments(ax, segments, color="tab:blue", lw=2.0, label=None):
    for k, s in enumerate(segments):
        ax.plot([s.p1[0], s.p2[0]], [s.p1[1], s.p2[1]], "-", color=color, lw=lw,
                label=label if k == 0 else None)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
