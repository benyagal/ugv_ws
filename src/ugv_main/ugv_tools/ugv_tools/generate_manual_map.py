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
# 1) EPULET ALAPMERETEK
# ============================================================
BUILDING_WIDTH_M = 12.9
BUILDING_LENGTH_M = 100.9

# ============================================================
# 2) TERKEP FELBONTAS ES UWB MAP FRAME ILLESZTES
# ============================================================
RESOLUTION_M_PER_PX = 0.05

# A kep BAL-ALSO pixelenek koordinataja a UWB `map` frame-ben. MERD LE PONTOSAN,
# hogy a rajzolt falak egybeessenek a UWB altal jelentett valos pozicioval.
ORIGIN_X_M = 0.0
ORIGIN_Y_M = 0.0
ORIGIN_YAW_RAD = 0.0

# ============================================================
# 3) VEGFALAK MENTI DOLGOZOI FOLYOSO (az epulet ket rovidebb oldalan)
# ============================================================
END_WALKWAY_DEPTH_M = 1.5   # vegfal -> ez a sav -> kerites -> csirketerulet
SIDE_FENCE_THICKNESS_M = 0.1  # a kerites akadalykent rajzolt vastagsaga
ROW_END_CLEARANCE_M = 1.0  # res a kerites es a sorok vege kozott, itt kel at a robot sorrol sorra

# ============================================================
# 4) SOROK (itato/etetu, valtakozva, itatoval kezdve es vegzodve)
# ============================================================
ROW_PATTERN = ["itato", "etetu", "itato", "etetu", "itato", "etetu", "itato"]

DRINKER_LINE_THICKNESS_M = 0.125   # 10-15 cm kozepe
FEEDER_LINE_THICKNESS_M = 0.05
FEEDER_BULGE_DIAMETER_M = 0.30
FEEDER_BULGE_SPACING_M = 1.0

# Sorok kozepvonalanak Y-pozicioja a teljes epuletszelesseghez (BUILDING_WIDTH_M)
# viszonyitva, a kozelebbi hosszanti faltol 0-nak veve (meterben). MERD LE
# PONTOSAN A HELYSZINEN -- amig nincs pontos ertek, a szkript egyenletesen
# elosztja a 7 sort.
ROW_Y_POSITIONS_M = None  # pl. [0.3, 2.0, 3.7, 5.4, 7.1, 8.8, 10.5]

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


def compute_row_y_positions():
    if ROW_Y_POSITIONS_M is not None:
        return ROW_Y_POSITIONS_M
    n_rows = len(ROW_PATTERN)
    step = BUILDING_WIDTH_M / (n_rows + 1)
    return [step * (i + 1) for i in range(n_rows)]


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
    fence_near_x = END_WALKWAY_DEPTH_M
    fence_far_x = BUILDING_LENGTH_M - END_WALKWAY_DEPTH_M
    rect(fence_near_x, 0, fence_near_x + SIDE_FENCE_THICKNESS_M, BUILDING_WIDTH_M)
    rect(fence_far_x - SIDE_FENCE_THICKNESS_M, 0, fence_far_x, BUILDING_WIDTH_M)

    # sorok Y-pozicioi a teljes epuletszelesseg menten
    row_y_positions = compute_row_y_positions()

    row_x_start = fence_near_x + ROW_END_CLEARANCE_M
    row_x_end = fence_far_x - ROW_END_CLEARANCE_M

    for row_type, y_offset in zip(ROW_PATTERN, row_y_positions):
        y_m = y_offset
        if row_type == "itato":
            half_t = DRINKER_LINE_THICKNESS_M / 2.0
            rect(row_x_start, y_m - half_t, row_x_end, y_m + half_t)
        elif row_type == "etetu":
            half_t = FEEDER_LINE_THICKNESS_M / 2.0
            rect(row_x_start, y_m - half_t, row_x_end, y_m + half_t)
            x = row_x_start
            while x <= row_x_end:
                circle(x, y_m, FEEDER_BULGE_DIAMETER_M)
                x += FEEDER_BULGE_SPACING_M
        else:
            raise ValueError(f"Ismeretlen sor tipus: {row_type}")

    return grid


def write_yaml(path):
    with open(path, "w") as f:
        f.write(f"image: {OUTPUT_PGM_NAME}\n")
        f.write("mode: trinary\n")
        f.write(f"resolution: {RESOLUTION_M_PER_PX}\n")
        f.write(f"origin: [{ORIGIN_X_M}, {ORIGIN_Y_M}, {ORIGIN_YAW_RAD}]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    grid = build_grid()
    Image.fromarray(grid, mode="L").save(OUTPUT_PGM)
    write_yaml(OUTPUT_YAML)
    print(f"Kesz: {OUTPUT_PGM} ({grid.shape[1]}x{grid.shape[0]} px), {OUTPUT_YAML}")


if __name__ == "__main__":
    main()
