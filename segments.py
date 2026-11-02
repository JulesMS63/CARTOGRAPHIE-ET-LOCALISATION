"""Segments 2D : extraction depuis un nuage de points + opérations géométriques.

Tolérances numériques (à citer dans le compte rendu) :
- EPS_LEN : 1e-9 m. Deux longueurs égales "à la tolérance près" si |L1 - L2| <= EPS_LEN.
            Une intersection est "à une extrémité" si elle est à moins de EPS_LEN d'un bout.
- EPS_SIN : 1e-9. Deux segments sont parallèles / colinéaires si
            |sin(angle entre eux)| <= EPS_SIN.
Ces valeurs sont des paramètres par défaut : toutes les fonctions les acceptent en argument.
"""

import math
from dataclasses import dataclass

import numpy as np

EPS_LEN = 1e-9
EPS_SIN = 1e-9


# --------------------------------------------------------------------------
# Segment
# --------------------------------------------------------------------------
@dataclass
class Segment:
    """Segment orienté de p1 vers p2 (coordonnées en mètres)."""

    p1: np.ndarray
    p2: np.ndarray
    weight: float = 1.0      # nombre d'observations cumulées (utile pour la carte)

    def __post_init__(self):
        self.p1 = np.asarray(self.p1, dtype=float)
        self.p2 = np.asarray(self.p2, dtype=float)

    @property
    def vector(self):
        return self.p2 - self.p1

    @property
    def length(self):
        return float(np.hypot(*self.vector))

    @property
    def direction(self):
        """Vecteur unitaire de p1 vers p2 (nul si segment dégénéré)."""
        length = self.length
        if length == 0.0:
            return np.zeros(2)
        return self.vector / length

    @property
    def angle(self):
        """Angle de la direction p1->p2 en radians, dans ]-pi, pi]."""
        vx, vy = self.vector
        return math.atan2(vy, vx)

    @property
    def midpoint(self):
        return (self.p1 + self.p2) / 2.0

    def as_tuple(self):
        return (float(self.p1[0]), float(self.p1[1]),
                float(self.p2[0]), float(self.p2[1]))

    def __repr__(self):
        x1, y1, x2, y2 = self.as_tuple()
        return f"Segment(({x1:.3f},{y1:.3f})->({x2:.3f},{y2:.3f}), L={self.length:.3f})"


def cross2(a, b):
    """Produit vectoriel 2D (scalaire) a x b."""
    return float(a[0] * b[1] - a[1] * b[0])


# --------------------------------------------------------------------------
# Distances
# --------------------------------------------------------------------------
def point_line_distance(point, seg):
    """Distance d'un point à la DROITE portée par le segment."""
    direction = seg.direction
    if seg.length == 0.0:
        return float(np.hypot(*(point - seg.p1)))
    offset = point - seg.p1
    distance = abs(cross2(direction, offset))
    return distance


def point_segment_distance(point, seg):
    """Distance d'un point au SEGMENT (extrémités comprises)."""
    vector = seg.vector
    length_sq = float(vector @ vector)
    if length_sq == 0.0:
        return float(np.hypot(*(point - seg.p1)))
    t = float((point - seg.p1) @ vector) / length_sq
    t = min(1.0, max(0.0, t))
    closest = seg.p1 + t * vector
    return float(np.hypot(*(point - closest)))


def segment_distance(a, b):
    """Distance minimale entre deux segments (0 s'ils se croisent)."""
    if intersection(a, b, tol=0.0) is not None:
        return 0.0
    candidates = [
        point_segment_distance(a.p1, b),
        point_segment_distance(a.p2, b),
        point_segment_distance(b.p1, a),
        point_segment_distance(b.p2, a),
    ]
    return min(candidates)


# --------------------------------------------------------------------------
# Angle, proximité, fusion
# --------------------------------------------------------------------------
def angle_between(a, b, oriented=False):
    """Angle entre deux segments, en radians.

    oriented=True  : angle entre les vecteurs p1->p2, dans [0, pi]
    oriented=False : angle entre les DROITES, dans [0, pi/2]
                     (deux murs sont "parallèles" même si leurs sens sont opposés)
    """
    va = a.vector
    vb = b.vector
    norm = np.hypot(*va) * np.hypot(*vb)
    if norm == 0.0:
        return 0.0
    # atan2(|cross|, dot) est plus précis que acos(dot/norm) près de 0 et de pi
    angle = math.atan2(abs(cross2(va, vb)), float(va @ vb))
    if oriented:
        return angle
    return min(angle, math.pi - angle)


def are_close(a, b, ang_tol=math.radians(10.0), dist_tol=0.05, gap_tol=0.10):
    """Dit si deux segments sont "proches" angulairement ET en position.

    ang_tol  : écart angulaire maximal entre les droites (rad)
    dist_tol : écart latéral maximal (m) : les extrémités du plus court doivent
               être à moins de dist_tol de la droite du plus long
    gap_tol  : trou maximal (m) entre les deux segments le long de leur direction
               (0 s'ils se recouvrent)
    La fonction est symétrique : are_close(a, b) == are_close(b, a).
    """
    if angle_between(a, b) > ang_tol:
        return False

    # on mesure l'écart latéral du plus court par rapport au plus long
    if a.length >= b.length:
        long_seg, short_seg = a, b
    else:
        long_seg, short_seg = b, a
    lateral = max(point_line_distance(short_seg.p1, long_seg),
                  point_line_distance(short_seg.p2, long_seg))
    if lateral > dist_tol:
        return False

    gap = segment_distance(a, b)
    return gap <= gap_tol


def merge(a, b):
    """Fusionne deux segments proches en un seul.

    1. droite moyenne = ajustement des moindres carrés totaux sur les 4 extrémités,
       pondérées par la longueur de chaque segment
    2. on projette les 4 extrémités sur cette droite
    3. le segment fusionné va de la projection la plus petite à la plus grande
    Le sens est celui de `a` (p1 -> p2 reste cohérent avec a).
    """
    points = np.array([a.p1, a.p2, b.p1, b.p2])
    weights = np.array([a.length, a.length, b.length, b.length], dtype=float)
    if weights.sum() == 0.0:
        weights = np.ones(4)

    centroid = (points * weights[:, None]).sum(axis=0) / weights.sum()
    centered = points - centroid
    covariance = (centered * weights[:, None]).T @ centered
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    direction = eigenvectors[:, np.argmax(eigenvalues)]

    # on garde le même sens que le segment a
    if float(direction @ a.vector) < 0.0:
        direction = -direction

    projections = centered @ direction
    start = centroid + projections.min() * direction
    end = centroid + projections.max() * direction
    merged = Segment(start, end, weight=a.weight + b.weight)
    return merged


# --------------------------------------------------------------------------
# Intersection et découpe
# --------------------------------------------------------------------------
def intersection(a, b, tol=EPS_LEN, sin_tol=EPS_SIN):
    """Intersection de deux segments.

    Retourne (point, t, u) où point = a.p1 + t*(a.p2-a.p1) = b.p1 + u*(b.p2-b.p1),
    ou None si :
      - les segments sont parallèles / colinéaires (|sin| <= sin_tol)
      - le point d'intersection des droites est hors d'un des segments
        (avec une marge de `tol` mètres)
    """
    da = a.vector
    db = b.vector
    len_a = np.hypot(*da)
    len_b = np.hypot(*db)
    if len_a == 0.0 or len_b == 0.0:
        return None

    denominator = cross2(da, db)
    if abs(denominator) <= sin_tol * len_a * len_b:
        return None                      # parallèles ou colinéaires

    w = b.p1 - a.p1
    t = cross2(w, db) / denominator
    u = cross2(w, da) / denominator

    margin_a = tol / len_a               # tolérance convertie en paramètre t
    margin_b = tol / len_b
    if t < -margin_a or t > 1.0 + margin_a:
        return None
    if u < -margin_b or u > 1.0 + margin_b:
        return None

    point = a.p1 + t * da
    return point, t, u


def cut_by(seg, cutter, tol=EPS_LEN, sin_tol=EPS_SIN):
    """Découpe `seg` s'il est traversé par `cutter`, en gardant le plus long côté.

    Règles du TP :
    - l'intersection doit être sur les DEUX segments
    - elle doit être STRICTEMENT à l'intérieur de `seg` (à plus de `tol` des extrémités),
      sinon `seg` est renvoyé inchangé
    - segments parallèles / colinéaires : jamais découpés
    - longueurs égales à `tol` près : on garde le côté qui contient seg.p1
    Retourne un nouveau Segment (ou `seg` lui-même s'il ne change pas).
    """
    length = seg.length
    if length <= tol:
        return seg

    hit = intersection(seg, cutter, tol=tol, sin_tol=sin_tol)
    if hit is None:
        return seg

    point, t, _u = hit
    dist_first = t * length               # longueur du côté qui contient p1
    dist_last = (1.0 - t) * length        # longueur du côté qui contient p2

    if dist_first <= tol or dist_last <= tol:
        return seg                        # intersection à une extrémité

    if abs(dist_first - dist_last) <= tol or dist_first > dist_last:
        return Segment(seg.p1, point, weight=seg.weight)    # côté de p1
    return Segment(point, seg.p2, weight=seg.weight)        # côté de p2


# --------------------------------------------------------------------------
# Extraction de segments depuis un nuage de points
# --------------------------------------------------------------------------
def fit_line_tls(points):
    """Droite des moindres carrés totaux. Retourne (centre, direction, rms)."""
    center = points.mean(axis=0)
    centered = points - center
    covariance = centered.T @ centered / len(points)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    direction = eigenvectors[:, np.argmax(eigenvalues)]
    rms = math.sqrt(max(0.0, float(eigenvalues.min())))
    return center, direction, rms


def _split_clusters(points, break_min, break_k, delta_theta):
    """Coupe la liste ordonnée de points là où deux voisins sont trop éloignés."""
    clusters = []
    start = 0
    for i in range(len(points) - 1):
        gap = float(np.hypot(*(points[i + 1] - points[i])))
        range_here = float(min(np.hypot(*points[i]), np.hypot(*points[i + 1])))
        threshold = max(break_min, break_k * range_here * delta_theta)
        if gap > threshold:
            clusters.append(points[start:i + 1])
            start = i + 1
    clusters.append(points[start:])
    return clusters


def _recursive_split(points, split_dist, min_points):
    """Split (Douglas-Peucker) : renvoie des listes de points quasi alignés."""
    pieces = []
    stack = [(0, len(points) - 1)]
    while stack:
        first, last = stack.pop()
        count = last - first + 1
        if count < min_points:
            continue

        chord = Segment(points[first], points[last])
        if chord.length == 0.0:
            continue

        inner = points[first + 1:last]
        if len(inner) == 0:
            pieces.append((first, last))
            continue

        distances = np.array([point_line_distance(p, chord) for p in inner])
        worst = int(np.argmax(distances))
        if distances[worst] > split_dist and count >= 2 * min_points - 1:
            cut = first + 1 + worst
            stack.append((first, cut))
            stack.append((cut, last))
        else:
            pieces.append((first, last))
    pieces.sort()
    return [points[a:b + 1] for a, b in pieces]


def extract_segments(points, split_dist=0.03, min_points=5, min_length=0.15,
                     break_min=0.15, break_k=4.0, max_rms=0.04):
    """Transforme un nuage de points ORDONNÉ (par angle) en liste de Segment.

    points      : tableau (N, 2) dans l'ordre de balayage du lidar
    split_dist  : un groupe est coupé si un point s'écarte de plus de ça de la corde (m)
    min_points  : nombre minimal de points pour garder un segment
    min_length  : longueur minimale d'un segment (m)
    break_min   : saut minimal entre deux points voisins pour couper un groupe (m)
    break_k     : le seuil de saut grandit avec la distance (k * r * delta_theta)
    max_rms     : erreur quadratique maximale du segment ajusté (m)
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) < min_points:
        return []

    # pas angulaire moyen entre deux points (sert à adapter le seuil de saut)
    angles = np.arctan2(points[:, 1], points[:, 0])
    steps = np.abs(np.diff(np.unwrap(angles)))
    delta_theta = float(np.median(steps)) if len(steps) else 0.01

    result = []
    clusters = _split_clusters(points, break_min, break_k, delta_theta)
    for cluster in clusters:
        if len(cluster) < min_points:
            continue
        pieces = _recursive_split(cluster, split_dist, min_points)
        for piece in pieces:
            if len(piece) < min_points:
                continue
            center, direction, rms = fit_line_tls(piece)
            if rms > max_rms:
                continue
            # extrémités = projection du premier et du dernier point sur la droite
            t_first = float((piece[0] - center) @ direction)
            t_last = float((piece[-1] - center) @ direction)
            start = center + t_first * direction
            end = center + t_last * direction
            seg = Segment(start, end, weight=1.0)
            if seg.length >= min_length:
                result.append(seg)
    return result


def merge_neighbors(segments, **close_kwargs):
    """Fusionne tous les segments proches (répète jusqu'à stabilité)."""
    segments = list(segments)
    changed = True
    while changed:
        changed = False
        for i in range(len(segments)):
            for j in range(i + 1, len(segments)):
                if are_close(segments[i], segments[j], **close_kwargs):
                    segments[i] = merge(segments[i], segments[j])
                    del segments[j]
                    changed = True
                    break
            if changed:
                break
    return segments
