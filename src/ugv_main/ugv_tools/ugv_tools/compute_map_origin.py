#!/usr/bin/env python3
"""Fits the ORIGIN_X_M/ORIGIN_Y_M/ORIGIN_YAW_RAD values for generate_manual_map.py
from measured reference-point pairs (building-frame vs. UWB map-frame).

Same method used for the existing dwm1001->map static TF calibration: 2D
Procrustes / Kabsch least-squares fit of a rigid transform (rotation + translation,
no scale) between two point sets.

Fill in REFERENCE_POINTS below with on-site measurements, then run:
    python3 compute_map_origin.py
"""
import numpy as np

# Minden sor: (building_x_m, building_y_m, map_x_m, map_y_m)
#   building_x/y : ismert koordinata a generate_manual_map.py geometriajaban
#                  (pl. epulet sarkai, kerites sarkai -- BUILDING_LENGTH_M/WIDTH_M-hez kepest)
#   map_x/y      : ugyanaz a pont, ahogy a UWB jelenti (RViz "Publish Point" a `map`
#                  frame-ben, vagy /uwb/pose leolvasas)
# MERD LE A HELYSZINEN -- legalabb 2, lehetoleg 3-4 pont kell a robusztus illesztéshez.
REFERENCE_POINTS = [
    (0.0, 0.0, 0.0, 0.0),
    (100.9, 0.0, 0.0, 0.0),
    (0.0, 12.9, 0.0, 0.0),
]


def fit_rigid_transform(building_pts, map_pts):
    """Legkisebb negyzetek illesztes: map_i ~= R @ building_i + t (Kabsch algoritmus)."""
    b = np.asarray(building_pts, dtype=float)
    m = np.asarray(map_pts, dtype=float)
    b_centroid = b.mean(axis=0)
    m_centroid = m.mean(axis=0)
    b_c = b - b_centroid
    m_c = m - m_centroid

    h = b_c.T @ m_c
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:  # tukrozes kizarasa, csak tiszta forgatas engedett
        vt[-1, :] *= -1
        r = vt.T @ u.T

    t = m_centroid - r @ b_centroid
    yaw = float(np.arctan2(r[1, 0], r[0, 0]))
    return t[0], t[1], yaw, r


def main():
    b_pts = [(p[0], p[1]) for p in REFERENCE_POINTS]
    m_pts = [(p[2], p[3]) for p in REFERENCE_POINTS]

    tx, ty, yaw, r = fit_rigid_transform(b_pts, m_pts)

    # residual hiba pontonkent (mennyire pontos az illesztes)
    residuals = []
    for (bx, by), (mx, my) in zip(b_pts, m_pts):
        pred = r @ np.array([bx, by]) + np.array([tx, ty])
        residuals.append(float(np.hypot(pred[0] - mx, pred[1] - my)))

    print(f"ORIGIN_X_M = {tx:.3f}")
    print(f"ORIGIN_Y_M = {ty:.3f}")
    print(f"ORIGIN_YAW_RAD = {yaw:.3f}  ({np.degrees(yaw):.1f} fok)")
    print(f"Residual hiba pontonkent (m): {[f'{e:.3f}' for e in residuals]}")
    print(f"Atlagos residual hiba: {np.mean(residuals):.3f} m")


if __name__ == "__main__":
    main()
