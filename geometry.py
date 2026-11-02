"""Outils géométriques de base (poses, rotations).

Conventions du module (voir "Utiliser les outils du module.md") :
- pose = (x, y, theta) avec x, y en mètres et theta en DEGRÉS
- axe X vers la droite, axe Y vers le haut
- angle positif = sens anti-horaire, theta = 0 -> le robot regarde vers +X
"""

import math

import numpy as np


def rotation_matrix(theta_deg):
    """Matrice de rotation 2x2 pour un angle en degrés (sens anti-horaire)."""
    theta_rad = math.radians(theta_deg)
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    matrix = np.array([[c, -s],
                       [s, c]])
    return matrix


def transform_points(points, pose):
    """Passe des points du repère robot/lidar au repère monde.

    points : tableau (N, 2) en mètres, dans le repère du lidar
    pose   : (x, y, theta_deg) du lidar dans le monde
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    x, y, theta_deg = pose
    rot = rotation_matrix(theta_deg)
    rotated = points @ rot.T          # équivaut à R * p pour chaque point
    translation = np.array([x, y])
    world_points = rotated + translation
    return world_points


def wrap_deg(angle_deg):
    """Ramène un angle en degrés dans [-180, 180[."""
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    return wrapped


def polar_to_cartesian(angles_rad, ranges):
    """(r, theta) -> (x, y), angles déjà en sens anti-horaire (math)."""
    x = ranges * np.cos(angles_rad)
    y = ranges * np.sin(angles_rad)
    points = np.column_stack((x, y))
    return points
