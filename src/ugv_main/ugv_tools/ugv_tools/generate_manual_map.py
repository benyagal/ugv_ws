#!/usr/bin/env python3
"""Generates a Nav2-compatible map.pgm/map.yaml pair from measured coop dimensions.

Run standalone (no ROS runtime needed): python3 generate_manual_map.py
All the numbers you are still missing are marked "MERD LE" below -- update them
once you have exact on-site measurements, everything else derives from them.
"""
import os

import numpy as np
from PIL import Image

# ============================================================
# 1) EPULET ALAPMERETEK (belso falsiktol belso falsikig)
# ============================================================
# Epulet-koordinatarendszer = a terkep `map` frame-je:
#   (0,0) = ORIGO SAROK (itt legyen az origo anchor);
#   +X a hosszanti fal menten a TAVOLI vegfal fele;
#   +Y a vegfal menten a SZEMKOZTI hosszanti fal fele.
# Az origo sarokban allva, +X fele nezve az epulet belseje BALRA van
# (kulonben tukrozott lenne a terkep).
BUILDING_WIDTH_M = 12.9
BUILDING_LENGTH_M = 100.9

# ============================================================
# 2) TERKEP FELBONTAS
# ============================================================
# A kep bal-also pixele = origo sarok, ezert a yaml origin mindig [0, 0, 0].
RESOLUTION_M_PER_PX = 0.05

# ============================================================
# 3) UWB ANCHOROK (MERD LE)
# ============================================================
# (nev, sarok, tav_vegfaltol_m, tav_hosszfaltol_m, magassag_m)
#   sarok: az a sarok, amihez merted:
#     "origo"      X=0,     Y=0
#     "tavoli_veg" X=hossz, Y=0
#     "szemkozti"  X=0,     Y=szelesseg
#     "atlos"      X=hossz, Y=szelesseg
#   tav_vegfaltol_m:   merolegesen a sarok VEGFALATOL (X irany)
#   tav_hosszfaltol_m: merolegesen a sarok HOSSZANTI FALATOL (Y irany)
#   magassag_m:        a padlotol (a DWM1001 konfiguraciohoz kell)
# Az ORIGIN_ANCHOR_NAME anchor lesz a DWM1001 rendszer (0,0) pontja.
ANCHORS = []  # pl. [("A0", "origo", 0.30, 0.20, 2.5), ("A1", "szemkozti", 0.30, 0.20, 2.5)]
ORIGIN_ANCHOR_NAME = "A0"

# ============================================================
# 4) VEGFALAK MENTI DOLGOZOI FOLYOSO (MERD LE, ket vegen kulon)
# ============================================================
# vegfal -> folyoso -> kerites -> csirketerulet
END_WALKWAY_DEPTH_NEAR_M = 1.5  # origo feloli vegfal (X=0) -> kerites
END_WALKWAY_DEPTH_FAR_M = 1.5   # tavoli vegfal (X=hossz) -> kerites
SIDE_FENCE_THICKNESS_M = 0.1  # a kerites akadalykent rajzolt vastagsaga
# res a kerites es a sorok vege kozott, itt kel at a robot sorrol sorra
ROW_END_CLEARANCE_NEAR_M = 1.0
ROW_END_CLEARANCE_FAR_M = 1.0

# ============================================================
# 5) SOROK (itato/etetu, valtakozva, itatoval kezdve es vegzodve)
# ============================================================
# Sorrend Y=0-tol (origo feloli hosszanti fal) a szemkozti fal fele.
ROW_PATTERN = ["itato", "etetu", "itato", "etetu", "itato", "etetu", "itato"]

DRINKER_LINE_THICKNESS_M = 0.125   # 10-15 cm kozepe
FEEDER_LINE_THICKNESS_M = 0.05
FEEDER_BULGE_DIAMETER_M = 0.30
FEEDER_FIRST_OFFSET_M = 0.0   # sor eleje (origo feloli vege) -> elso etetu KOZEPE (MERD LE)
FEEDER_BULGE_SPACING_M = 1.0  # etetok kozott, kozeptol kozepig (MERD LE)

# Y=0 fal -> 1., 2., ... sor KOZEPVONALA, ROW_PATTERN sorrendjeben (MERD LE).
# None eseten egyenletes elosztas.
ROW_Y_POSITIONS_M = None  # pl. [1.1, 3.2, 5.3, 7.4, 9.6, 10.8, 11.4]

# ============================================================
# 5) KIMENET
# ============================================================
# realpath, hogy symlink-installed (ros2 run) futtataskor is a forras melletti
# maps/ mappaba kerulhessen, ne a futtatasi konyvtarba
_PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
OUTPUT_DIR = os.path.join(_PACKAGE_ROOT, "maps")
OUTPUT_PGM_NAME = "coop_map.pgm"
OUTPUT_YAML_NAME = "coop_map.yaml"
OUTPUT_PGM = os.path.join(OUTPUT_DIR, OUTPUT_PGM_NAME)
OUTPUT_YAML = os.path.join(OUTPUT_DIR, OUTPUT_YAML_NAME)

FREE, OCCUPIED = 255, 0


_CORNERS = {
    "origo": (False, False),
    "tavoli_veg": (True, False),
    "szemkozti": (False, True),
    "atlos": (True, True),
}


def compute_row_y_positions():
    if ROW_Y_POSITIONS_M is None:
        n_rows = len(ROW_PATTERN)
        step = BUILDING_WIDTH_M / (n_rows + 1)
        return [step * (i + 1) for i in range(n_rows)]
    if len(ROW_Y_POSITIONS_M) != len(ROW_PATTERN):
        raise ValueError(f"ROW_Y_POSITIONS_M-nek {len(ROW_PATTERN)} erteke kell legyen (ROW_PATTERN)")
    if any(b <= a for a, b in zip(ROW_Y_POSITIONS_M, ROW_Y_POSITIONS_M[1:])):
        raise ValueError("ROW_Y_POSITIONS_M-nek szigoruan novekvonek kell lennie")
    if ROW_Y_POSITIONS_M[0] <= 0 or ROW_Y_POSITIONS_M[-1] >= BUILDING_WIDTH_M:
        raise ValueError(f"ROW_Y_POSITIONS_M ertekeinek 0 es {BUILDING_WIDTH_M} kozott kell lenniuk")
    return list(ROW_Y_POSITIONS_M)


def fence_x_positions():
    return END_WALKWAY_DEPTH_NEAR_M, BUILDING_LENGTH_M - END_WALKWAY_DEPTH_FAR_M


def row_x_range():
    fence_near_x, fence_far_x = fence_x_positions()
    return fence_near_x + ROW_END_CLEARANCE_NEAR_M, fence_far_x - ROW_END_CLEARANCE_FAR_M


def feeder_x_positions():
    row_x_start, row_x_end = row_x_range()
    positions = []
    x = row_x_start + FEEDER_FIRST_OFFSET_M
    while x <= row_x_end:
        positions.append(x)
        x += FEEDER_BULGE_SPACING_M
    return positions


def anchor_building_positions():
    """{nev: (x, y, z)} az epulet (= map) koordinatarendszerben."""
    result = {}
    for name, corner, d_end, d_side, height in ANCHORS:
        if corner not in _CORNERS:
            raise ValueError(f"{name}: ismeretlen sarok '{corner}', lehet: {list(_CORNERS)}")
        far_end, far_side = _CORNERS[corner]
        x = BUILDING_LENGTH_M - d_end if far_end else d_end
        y = BUILDING_WIDTH_M - d_side if far_side else d_side
        result[name] = (x, y, height)
    return result


def print_anchor_setup():
    anchors = anchor_building_positions()
    if not anchors:
        print("ANCHORS ures - anchor koordinatak es UWB TF kihagyva.")
        return
    if ORIGIN_ANCHOR_NAME not in anchors:
        raise ValueError(f"ORIGIN_ANCHOR_NAME '{ORIGIN_ANCHOR_NAME}' nincs az ANCHORS-ban")
    ox, oy, _ = anchors[ORIGIN_ANCHOR_NAME]
    print("\nDWM1001 anchor koordinatak (ezeket kell beallitani az anchorokon):")
    print(f"  {'nev':<6}{'x [m]':>9}{'y [m]':>9}{'z [m]':>9}")
    for name, (x, y, z) in anchors.items():
        print(f"  {name:<6}{x - ox:>9.3f}{y - oy:>9.3f}{z:>9.3f}")
    print("\nbringup_localization_uwb.launch.py, dwm1001_to_map_tf arguments:")
    print(f"  ['{ox:.3f}', '{oy:.3f}', '0', '0', '0', '0', 'map', 'dwm1001']")


def build_grid():
    w_px = int(round(BUILDING_LENGTH_M / RESOLUTION_M_PER_PX))
    h_px = int(round(BUILDING_WIDTH_M / RESOLUTION_M_PER_PX))
    grid = np.full((h_px, w_px), FREE, dtype=np.uint8)

    def px_x(x_m):
        return int(round(x_m / RESOLUTION_M_PER_PX))

    def px_y(y_m):
        # kep 0. sora = legfelso = legnagyobb szelesseg-koordinata
        return h_px - 1 - int(round(y_m / RESOLUTION_M_PER_PX))

    def rect(x1_m, y1_m, x2_m, y2_m):
        c1, c2 = sorted((px_x(x1_m), px_x(x2_m)))
        r1, r2 = sorted((px_y(y1_m), px_y(y2_m)))
        grid[max(r1, 0):min(r2 + 1, h_px), max(c1, 0):min(c2 + 1, w_px)] = OCCUPIED

    def circle(cx_m, cy_m, diameter_m):
        radius_px = diameter_m / 2.0 / RESOLUTION_M_PER_PX
        cx_px, cy_px = px_x(cx_m), px_y(cy_m)
        r0, r1 = int(cy_px - radius_px), int(cy_px + radius_px) + 1
        c0, c1 = int(cx_px - radius_px), int(cx_px + radius_px) + 1
        rows, cols = np.ogrid[max(r0, 0):min(r1, h_px), max(c0, 0):min(c1, w_px)]
        mask = (rows - cy_px) ** 2 + (cols - cx_px) ** 2 <= radius_px ** 2
        grid[max(r0, 0):min(r1, h_px), max(c0, 0):min(c1, w_px)][mask] = OCCUPIED

    # kulso falak
    rect(0, 0, BUILDING_LENGTH_M, SIDE_FENCE_THICKNESS_M)
    rect(0, BUILDING_WIDTH_M - SIDE_FENCE_THICKNESS_M, BUILDING_LENGTH_M, BUILDING_WIDTH_M)
    rect(0, 0, SIDE_FENCE_THICKNESS_M, BUILDING_WIDTH_M)
    rect(BUILDING_LENGTH_M - SIDE_FENCE_THICKNESS_M, 0, BUILDING_LENGTH_M, BUILDING_WIDTH_M)

    # dolgozoi folyoso es csirketerulet kozotti keritesek (az epulet ket vegen)
    fence_near_x, fence_far_x = fence_x_positions()
    rect(fence_near_x, 0, fence_near_x + SIDE_FENCE_THICKNESS_M, BUILDING_WIDTH_M)
    rect(fence_far_x - SIDE_FENCE_THICKNESS_M, 0, fence_far_x, BUILDING_WIDTH_M)

    row_y_positions = compute_row_y_positions()
    row_x_start, row_x_end = row_x_range()

    for row_type, y_offset in zip(ROW_PATTERN, row_y_positions):
        y_m = y_offset
        if row_type == "itato":
            half_t = DRINKER_LINE_THICKNESS_M / 2.0
            rect(row_x_start, y_m - half_t, row_x_end, y_m + half_t)
        elif row_type == "etetu":
            half_t = FEEDER_LINE_THICKNESS_M / 2.0
            rect(row_x_start, y_m - half_t, row_x_end, y_m + half_t)
            for x in feeder_x_positions():
                circle(x, y_m, FEEDER_BULGE_DIAMETER_M)
        else:
            raise ValueError(f"Ismeretlen sor tipus: {row_type}")

    return grid


def write_yaml(path):
    with open(path, "w") as f:
        f.write(f"image: {OUTPUT_PGM_NAME}\n")
        f.write("mode: trinary\n")
        f.write(f"resolution: {RESOLUTION_M_PER_PX}\n")
        f.write("origin: [0.0, 0.0, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    grid = build_grid()
    Image.fromarray(grid, mode="L").save(OUTPUT_PGM)
    write_yaml(OUTPUT_YAML)
    print(f"Kesz: {OUTPUT_PGM} ({grid.shape[1]}x{grid.shape[0]} px), {OUTPUT_YAML}")

    rows = compute_row_y_positions()
    print(f"Ellenorzes: utolso sor kozepvonala -> szemkozti fal = "
          f"{BUILDING_WIDTH_M - rows[-1]:.2f} m (merd le, egyezzen)")
    print_anchor_setup()


if __name__ == "__main__":
    main()
