#!/usr/bin/env python3
"""Annotated schematic drawing of the manual coop map, with distances labeled.

Reuses the parameters/geometry from generate_manual_map.py (single source of
truth) and renders a scaled-up PIL image with dimension lines and labels, so
the layout can be visually reviewed without needing Nav2/RViz.

Run standalone: python3 visualize_manual_map.py
"""
import os

from PIL import Image, ImageDraw, ImageFont

try:
    from ugv_tools import generate_manual_map as gm
except ImportError:  # kozvetlen "python3 visualize_manual_map.py" futtataskor
    import generate_manual_map as gm

# felskalazas a nyers terkep pixelmeretehez kepest, hogy a feliratok olvashatok legyenek
SCALE = 6
MARGIN_PX = 90  # hely a meretvonalaknak/feliratoknak a kep szelein
FENCE_COLOR = (200, 30, 30)
DRINKER_COLOR = (30, 90, 200)
FEEDER_COLOR = (30, 150, 60)
DIM_COLOR = (0, 0, 0)
WALL_COLOR = (0, 0, 0)

OUTPUT_PNG_NAME = "coop_map_diagram.png"
OUTPUT_PNG = os.path.join(gm.OUTPUT_DIR, OUTPUT_PNG_NAME)


def m_to_px(x_m, y_m, w_m_px, h_m_px):
    """Meter koordinatat kep-pixelre valt (X jobbra, Y felfele nő), margoval eltolva."""
    x_px = MARGIN_PX + x_m / gm.RESOLUTION_M_PER_PX * SCALE
    y_px = MARGIN_PX + h_m_px - y_m / gm.RESOLUTION_M_PER_PX * SCALE
    return x_px, y_px


def _font(size):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except OSError:
        return ImageFont.load_default()


def draw_dimension(draw, x1, y1, x2, y2, label, font, offset=18):
    """Egyszeru meretvonal ket pont kozott, kozepre irt felirattal."""
    draw.line([(x1, y1), (x2, y2)], fill=DIM_COLOR, width=2)
    # vegzo "kapcsok"
    for x, y in ((x1, y1), (x2, y2)):
        draw.line([(x - 5, y - 5), (x + 5, y + 5)], fill=DIM_COLOR, width=2)
        draw.line([(x - 5, y + 5), (x + 5, y - 5)], fill=DIM_COLOR, width=2)
    mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
    bbox = draw.textbbox((0, 0), label, font=font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    if abs(y2 - y1) > abs(x2 - x1):  # fuggoleges meretvonal -> felirat oldalra
        draw.text((mid_x - tw - 8, mid_y - th / 2), label, fill=DIM_COLOR, font=font)
    else:
        draw.text((mid_x - tw / 2, mid_y - offset), label, fill=DIM_COLOR, font=font)


def render():
    w_m_px = gm.BUILDING_LENGTH_M / gm.RESOLUTION_M_PER_PX * SCALE
    h_m_px = gm.BUILDING_WIDTH_M / gm.RESOLUTION_M_PER_PX * SCALE
    img = Image.new("RGB", (int(w_m_px + 2 * MARGIN_PX), int(h_m_px + 2 * MARGIN_PX)), "white")
    draw = ImageDraw.Draw(img)
    font = _font(16)
    font_small = _font(13)

    def P(x_m, y_m):
        return m_to_px(x_m, y_m, w_m_px, h_m_px)

    # epulet korvonal
    draw.rectangle([P(0, gm.BUILDING_WIDTH_M), P(gm.BUILDING_LENGTH_M, 0)], outline=WALL_COLOR, width=3)

    # vegfali dolgozoi folyoso keritesek
    fence_near_x = gm.END_WALKWAY_DEPTH_M
    fence_far_x = gm.BUILDING_LENGTH_M - gm.END_WALKWAY_DEPTH_M
    for fx in (fence_near_x, fence_far_x):
        draw.line([P(fx, 0), P(fx, gm.BUILDING_WIDTH_M)], fill=FENCE_COLOR, width=3)

    # sorok (a kerites es a sorok vege kozotti res: ROW_END_CLEARANCE_M, itt kel at a robot)
    row_x_start = fence_near_x + gm.ROW_END_CLEARANCE_M
    row_x_end = fence_far_x - gm.ROW_END_CLEARANCE_M
    row_y_positions = gm.compute_row_y_positions()
    for row_type, y_m in zip(gm.ROW_PATTERN, row_y_positions):
        color = DRINKER_COLOR if row_type == "itato" else FEEDER_COLOR
        draw.line([P(row_x_start, y_m), P(row_x_end, y_m)], fill=color, width=2)
        if row_type == "etetu":
            r_px = gm.FEEDER_BULGE_DIAMETER_M / 2.0 / gm.RESOLUTION_M_PER_PX * SCALE
            x = row_x_start
            while x <= row_x_end:
                cx, cy = P(x, y_m)
                draw.ellipse([cx - r_px, cy - r_px, cx + r_px, cy + r_px], outline=FEEDER_COLOR, width=2)
                x += gm.FEEDER_BULGE_SPACING_M

    # --- meretvonalak ---
    # teljes hossz (also szelen kivul)
    y_dim = P(0, 0)[1] + 40
    x0, x1 = P(0, 0)[0], P(gm.BUILDING_LENGTH_M, 0)[0]
    draw_dimension(draw, x0, y_dim, x1, y_dim, f"{gm.BUILDING_LENGTH_M} m (hossz)", font)

    # teljes szelesseg (bal szelen kivul)
    x_dim = P(0, 0)[0] - 45
    y0, y1 = P(0, 0)[1], P(0, gm.BUILDING_WIDTH_M)[1]
    draw_dimension(draw, x_dim, y0, x_dim, y1, f"{gm.BUILDING_WIDTH_M} m (szelesseg)", font)

    # vegfali folyoso melysege (mindket vegen)
    for fx, label_side in ((fence_near_x, "kozel"), (fence_far_x, "tavoli")):
        x_a = P(0 if fx == fence_near_x else gm.BUILDING_LENGTH_M, 0)[0]
        x_b = P(fx, 0)[0]
        y_dim2 = P(0, 0)[1] + 70
        draw_dimension(draw, x_a, y_dim2, x_b, y_dim2, f"{gm.END_WALKWAY_DEPTH_M} m", font_small)

    # sorok tavolsaga (elso ket sor kozott, peldakent)
    if len(row_y_positions) >= 2:
        spacing = row_y_positions[1] - row_y_positions[0]
        x_dim2 = P(row_x_start, 0)[0] - 20
        y_a, y_b = P(0, row_y_positions[0])[1], P(0, row_y_positions[1])[1]
        draw_dimension(draw, x_dim2, y_a, x_dim2, y_b, f"{spacing:.2f} m", font_small)

    # robot-atjaro res a kerites es a sorok vege kozott (kozeli oldalon, peldakent)
    y_dim3 = P(0, row_y_positions[0])[1] - 25
    x_a, x_b = P(fence_near_x, 0)[0], P(row_x_start, 0)[0]
    draw_dimension(draw, x_a, y_dim3, x_b, y_dim3, f"{gm.ROW_END_CLEARANCE_M} m atjaro", font_small)

    # etetu bura atmero (elso etetu sor elso bura melle irva)
    for row_type, y_m in zip(gm.ROW_PATTERN, row_y_positions):
        if row_type == "etetu":
            cx, cy = P(row_x_start, y_m)
            r_px = gm.FEEDER_BULGE_DIAMETER_M / 2.0 / gm.RESOLUTION_M_PER_PX * SCALE
            draw.text((cx + r_px + 6, cy - 8), f"o{gm.FEEDER_BULGE_DIAMETER_M * 100:.0f} cm", fill=FEEDER_COLOR, font=font_small)
            break

    # itato vastagsag (elso itato sor elejere irva)
    for row_type, y_m in zip(gm.ROW_PATTERN, row_y_positions):
        if row_type == "itato":
            cx, cy = P(row_x_start, y_m)
            draw.text((cx + 6, cy + 6), f"{gm.DRINKER_LINE_THICKNESS_M * 100:.0f} cm vastag", fill=DRINKER_COLOR, font=font_small)
            break

    return img


def main():
    os.makedirs(gm.OUTPUT_DIR, exist_ok=True)
    img = render()
    img.save(OUTPUT_PNG)
    print(f"Kesz: {OUTPUT_PNG} ({img.width}x{img.height} px)")


if __name__ == "__main__":
    main()
