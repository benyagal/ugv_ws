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
ANCHOR_COLOR = (230, 120, 0)

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
    _, fence_far_x = gm.fence_x_positions()
    draw.line([P(x, y) for x, y in gm.NEAR_FENCE_POINTS_M], fill=FENCE_COLOR, width=3, joint="curve")
    draw.line([P(fence_far_x, 0), P(fence_far_x, gm.BUILDING_WIDTH_M)], fill=FENCE_COLOR, width=3)

    # sorok (a kerites es a sorok vege kozotti res: ROW_END_CLEARANCE_*, itt kel at a robot)
    row_y_positions = gm.compute_row_y_positions()
    for row_type, y_m in zip(gm.ROW_PATTERN, row_y_positions):
        row_x_start, row_x_end = gm.row_x_range(y_m)
        color = DRINKER_COLOR if row_type == "itato" else FEEDER_COLOR
        draw.line([P(row_x_start, y_m), P(row_x_end, y_m)], fill=color, width=2)
        if row_type == "etetu":
            rx_px = gm.FEEDER_SIZE_ALONG_ROW_M / 2.0 / gm.RESOLUTION_M_PER_PX * SCALE
            ry_px = gm.FEEDER_SIZE_ACROSS_ROW_M / 2.0 / gm.RESOLUTION_M_PER_PX * SCALE
            for x in gm.feeder_x_positions(y_m):
                cx, cy = P(x, y_m)
                draw.ellipse([cx - rx_px, cy - ry_px, cx + rx_px, cy + ry_px], outline=FEEDER_COLOR, width=2)

    # UWB anchorok (csak a diagramon, a pgm-ben nem akadalyok)
    for name, (ax, ay, _) in gm.anchor_building_positions().items():
        cx, cy = P(ax, ay)
        draw.rectangle([cx - 7, cy - 7, cx + 7, cy + 7], fill=ANCHOR_COLOR)
        label = f"{name} (origo)" if name == gm.ORIGIN_ANCHOR_NAME else name
        draw.text((cx + 10, cy - 8), label, fill=ANCHOR_COLOR, font=font)

    # origo sarok es tengelyek
    ox, oy = P(0, 0)
    draw.line([(ox, oy), (ox + 60, oy)], fill=WALL_COLOR, width=3)
    draw.line([(ox, oy), (ox, oy - 60)], fill=WALL_COLOR, width=3)
    draw.text((ox + 64, oy - 10), "+X", fill=WALL_COLOR, font=font)
    draw.text((ox - 8, oy - 82), "+Y", fill=WALL_COLOR, font=font)
    draw.text((ox - 60, oy + 8), "(0,0)", fill=WALL_COLOR, font=font_small)

    # --- meretvonalak ---
    # teljes hossz (also szelen kivul)
    y_dim = P(0, 0)[1] + 40
    x0, x1 = P(0, 0)[0], P(gm.BUILDING_LENGTH_M, 0)[0]
    draw_dimension(draw, x0, y_dim, x1, y_dim, f"{gm.BUILDING_LENGTH_M} m (hossz)", font)

    # teljes szelesseg (bal szelen kivul)
    x_dim = P(0, 0)[0] - 45
    y0, y1 = P(0, 0)[1], P(0, gm.BUILDING_WIDTH_M)[1]
    draw_dimension(draw, x_dim, y0, x_dim, y1, f"{gm.BUILDING_WIDTH_M} m (szelesseg)", font)

    # vegfali folyoso melysege (mindket vegen kulon; origo felol a kerites ket egyenes szakaszahoz)
    y_dim2 = P(0, 0)[1] + 62
    near_depths = sorted({x for x, _ in gm.NEAR_FENCE_POINTS_M})
    for i, depth in enumerate(near_depths):
        y_d = y_dim2 + 22 * i
        draw_dimension(draw, P(0, 0)[0], y_d, P(depth, 0)[0], y_d, f"{depth:.3g} m", font_small)
    draw_dimension(draw, P(fence_far_x, 0)[0], y_dim2, P(gm.BUILDING_LENGTH_M, 0)[0], y_dim2,
                   f"{gm.END_WALKWAY_DEPTH_FAR_M} m", font_small)

    # sorok tavolsaga (minden szomszedos par kozott)
    x_dim2 = P(min(gm.row_x_range(y)[0] for y in row_y_positions), 0)[0] - 20
    for y_lo, y_hi in zip(row_y_positions, row_y_positions[1:]):
        y_a, y_b = P(0, y_lo)[1], P(0, y_hi)[1]
        draw_dimension(draw, x_dim2, y_a, x_dim2, y_b, f"{y_hi - y_lo:.2f} m", font_small)
    # fal -> elso sor
    draw_dimension(draw, x_dim2, P(0, 0)[1], x_dim2, P(0, row_y_positions[0])[1],
                   f"{row_y_positions[0]:.2f} m", font_small)

    # robot-atjaro res a kerites es a sorok vege kozott (mindket vegen)
    y_dim3 = P(0, row_y_positions[0])[1] - 25
    row_x_start, row_x_end = gm.row_x_range(row_y_positions[0])
    draw_dimension(draw, P(gm.near_fence_x_at(row_y_positions[0]), 0)[0], y_dim3, P(row_x_start, 0)[0], y_dim3,
                   f"{gm.ROW_END_CLEARANCE_NEAR_M} m atjaro", font_small)
    draw_dimension(draw, P(row_x_end, 0)[0], y_dim3, P(fence_far_x, 0)[0], y_dim3,
                   f"{gm.ROW_END_CLEARANCE_FAR_M} m atjaro", font_small)

    # etetu bura atmero (elso etetu sor elso bura melle irva)
    for row_type, y_m in zip(gm.ROW_PATTERN, row_y_positions):
        if row_type == "etetu":
            cx, cy = P(gm.row_x_range(y_m)[0], y_m)
            rx_px = gm.FEEDER_SIZE_ALONG_ROW_M / 2.0 / gm.RESOLUTION_M_PER_PX * SCALE
            draw.text((cx + rx_px + 6, cy - 8),
                      f"{gm.FEEDER_SIZE_ALONG_ROW_M * 100:.0f}x{gm.FEEDER_SIZE_ACROSS_ROW_M * 100:.0f} cm",
                      fill=FEEDER_COLOR, font=font_small)
            break

    # itato vastagsag (elso itato sor elejere irva)
    for row_type, y_m in zip(gm.ROW_PATTERN, row_y_positions):
        if row_type == "itato":
            cx, cy = P(gm.row_x_range(y_m)[0], y_m)
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
