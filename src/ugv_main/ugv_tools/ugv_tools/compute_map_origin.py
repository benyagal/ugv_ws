#!/usr/bin/env python3
"""Fits the map->dwm1001 static TF (bringup_localization_uwb.launch.py) from
measured reference-point pairs (building/map frame vs. raw DWM1001 frame).

The manual map is drawn in the building frame with origin [0,0,0] (see
generate_manual_map.py), so the map frame == building frame and only the
UWB->map transform has to be calibrated.

2D Procrustes / Kabsch least-squares fit of a rigid transform (rotation +
translation, no scale): p_map ~= R(yaw) @ p_dwm + t.

Fill in REFERENCE_POINTS below with on-site measurements, then run:
    python3 compute_map_origin.py
"""
import numpy as np

# Minden sor: (building_x_m, building_y_m, dwm_x_m, dwm_y_m)
#   building_x/y : a pont merszalaggal mert helye az epulet-koordinatarendszerben
#                  (origo sarok = (0,0), X a hosszanti fal menten, Y a vegfal menten;
#                  ld. generate_manual_map.py)
#   dwm_x/y      : ugyanitt allo tag nyers pozicioja: `ros2 topic echo /uwb/point_raw`
#                  (par masodperc atlaga)
# MERD LE A HELYSZINEN -- legalabb 2, lehetoleg 3-4 egymastol tavoli pont kell.
REFERENCE_POINTS = [
    (0.0, 0.0, 0.0, 0.0),
    (100.9, 0.0, 0.0, 0.0),
    (0.0, 12.9, 0.0, 0.0),
]


def fit_rigid_transform(src_pts, dst_pts):
    """Legkisebb negyzetek illesztes: dst_i ~= R @ src_i + t (Kabsch algoritmus)."""
    b = np.asarray(src_pts, dtype=float)
    m = np.asarray(dst_pts, dtype=float)
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
    map_pts = [(p[0], p[1]) for p in REFERENCE_POINTS]
    dwm_pts = [(p[2], p[3]) for p in REFERENCE_POINTS]

    tx, ty, yaw, r = fit_rigid_transform(dwm_pts, map_pts)

    # residual hiba pontonkent (mennyire pontos az illesztes)
    residuals = []
    for (dx, dy), (mx, my) in zip(dwm_pts, map_pts):
        pred = r @ np.array([dx, dy]) + np.array([tx, ty])
        residuals.append(float(np.hypot(pred[0] - mx, pred[1] - my)))

    print("bringup_localization_uwb.launch.py, dwm1001_to_map_tf arguments:")
    print(f"  ['{tx:.3f}', '{ty:.3f}', '0', '{yaw:.3f}', '0', '0', 'map', 'dwm1001']"
          f"  (yaw = {np.degrees(yaw):.1f} fok)")
    print(f"Residual hiba pontonkent (m): {[f'{e:.3f}' for e in residuals]}")
    print(f"Atlagos residual hiba: {np.mean(residuals):.3f} m")


if __name__ == "__main__":
    main()
