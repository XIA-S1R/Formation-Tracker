#!/usr/bin/env python3
import ast
import math
from pathlib import Path

from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[3]
EVASION_SCRIPT = ROOT / "src/planning/planning/scripts/real_field_evasion.py"
OUT = ROOT / "src/real_uav_bridge/maps/generated/real_field_evasion_preview.png"


FIELD = {
    "x_min": -3.3,
    "x_max": 3.0,
    "y_min": -2.8,
    "y_max": 2.5,
}
OBSTACLES = [
    ("pillar_0", 1.5, 2.1, 0.0, 1.2),
    ("pillar_1", -0.4, 0.2, -1.6, -0.4),
    ("pillar_2", -2.2, -1.6, -0.5, 0.7),
]
STARTS = [
    ("target", 0.00, 2.10, (210, 35, 35)),
    ("drone0", 0.00, 1.15, (0, 95, 210)),
    ("drone1", -0.55, 1.85, (0, 145, 70)),
    ("drone2", 0.55, 1.85, (230, 140, 0)),
]


def load_waypoints():
    src = EVASION_SCRIPT.read_text(encoding="utf-8")
    mod = ast.parse(src)
    node = next(
        n.value
        for n in mod.body
        if isinstance(n, ast.Assign)
        and any(getattr(t, "id", None) == "WAYPOINTS" for t in n.targets)
    )
    return eval(compile(ast.Expression(node), "<waypoints>", "eval"), {"math": math})


def main():
    waypoints = load_waypoints()
    scale = 120
    margin = 90
    width = int(round((FIELD["x_max"] - FIELD["x_min"]) * scale)) + 2 * margin
    height = int(round((FIELD["y_max"] - FIELD["y_min"]) * scale)) + 2 * margin
    img = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(img)

    def px(x, y):
        return (
            int(round(margin + (x - FIELD["x_min"]) * scale)),
            int(round(height - margin - (y - FIELD["y_min"]) * scale)),
        )

    draw.rectangle((*px(FIELD["x_min"], FIELD["y_max"]), *px(FIELD["x_max"], FIELD["y_min"])),
                   outline=(20, 20, 20), width=3)
    gx = math.ceil(FIELD["x_min"] * 2) / 2.0
    while gx <= FIELD["x_max"] + 1e-9:
        draw.line((px(gx, FIELD["y_min"]), px(gx, FIELD["y_max"])),
                  fill=(120, 120, 120) if abs(gx) < 1e-9 else (215, 215, 215),
                  width=2 if abs(gx) < 1e-9 else 1)
        gx += 0.5
    gy = math.ceil(FIELD["y_min"] * 2) / 2.0
    while gy <= FIELD["y_max"] + 1e-9:
        draw.line((px(FIELD["x_min"], gy), px(FIELD["x_max"], gy)),
                  fill=(120, 120, 120) if abs(gy) < 1e-9 else (215, 215, 215),
                  width=2 if abs(gy) < 1e-9 else 1)
        gy += 0.5

    colors = [(224, 87, 71), (69, 137, 209), (79, 170, 91)]
    for i, (name, x0, x1, y0, y1) in enumerate(OBSTACLES):
        draw.rectangle((*px(x0, y1), *px(x1, y0)), fill=colors[i], outline=(0, 0, 0), width=3)
        c = px((x0 + x1) / 2.0, (y0 + y1) / 2.0)
        draw.text((c[0] - 26, c[1] - 8), name, fill=(255, 255, 255))

    path = [px(w[1], w[2]) for w in waypoints]
    draw.line(path, fill=(20, 20, 20), width=5, joint="curve")
    draw.line(path, fill=(255, 210, 0), width=3, joint="curve")
    for idx, w in enumerate(waypoints):
        p = px(w[1], w[2])
        r = 5
        draw.ellipse((p[0] - r, p[1] - r, p[0] + r, p[1] + r), fill=(0, 0, 0))
        draw.text((p[0] + 7, p[1] - 14), str(idx), fill=(0, 0, 0))

    for name, x, y, color in STARTS:
        p = px(x, y)
        r = 8
        draw.ellipse((p[0] - r, p[1] - r, p[0] + r, p[1] + r), fill=color, outline=(0, 0, 0), width=2)
        draw.text((p[0] + 10, p[1] - 10), name, fill=(0, 0, 0))

    draw.text((margin, 20), "Real-field target evasion preview with starts", fill=(0, 0, 0))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    img.save(OUT)
    print(OUT)


if __name__ == "__main__":
    main()
