#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    Image = None
    ImageDraw = None
    ImageFont = None


def frange_inclusive(vmin, vmax, step):
    count = int(math.floor((vmax - vmin) / step + 0.5))
    values = [vmin + i * step for i in range(count + 1)]
    if not values or abs(values[-1] - vmax) > 1e-9:
        values.append(vmax)
    return [round(v, 6) for v in values if v <= vmax + 1e-9]


def obstacle_bounds(obstacle, size):
    x_rf, y_rf = obstacle["right_front_corner_m"]
    return {
        "name": obstacle.get("name", "obstacle"),
        "x_min": float(x_rf) - float(size["x"]),
        "x_max": float(x_rf),
        "y_min": float(y_rf) - float(size["y"]),
        "y_max": float(y_rf),
        "z_min": 0.0,
        "z_max": float(size["z"]),
        "right_front": (float(x_rf), float(y_rf)),
    }


def generate_points(bounds, step):
    points = []
    for box in bounds:
        xs = frange_inclusive(box["x_min"], box["x_max"], step)
        ys = frange_inclusive(box["y_min"], box["y_max"], step)
        zs = frange_inclusive(box["z_min"], box["z_max"], step)
        for x in xs:
            for y in ys:
                for z in zs:
                    points.append((x, y, z))
    return points


def write_csv(path, points, bounds):
    with path.open("w", newline="", encoding="utf-8") as f:
        f.write("# measured static obstacle point cloud\n")
        f.write("# columns: x y z, unit: meter, frame: world/ENU\n")
        for box in bounds:
            f.write(
                "# {name}: x=[{x_min:.3f},{x_max:.3f}], "
                "y=[{y_min:.3f},{y_max:.3f}], z=[{z_min:.3f},{z_max:.3f}], "
                "right_front=({rf_x:.3f},{rf_y:.3f})\n".format(
                    name=box["name"],
                    x_min=box["x_min"],
                    x_max=box["x_max"],
                    y_min=box["y_min"],
                    y_max=box["y_max"],
                    z_min=box["z_min"],
                    z_max=box["z_max"],
                    rf_x=box["right_front"][0],
                    rf_y=box["right_front"][1],
                )
            )
        writer = csv.writer(f, delimiter=" ")
        for p in points:
            writer.writerow([f"{p[0]:.3f}", f"{p[1]:.3f}", f"{p[2]:.3f}"])


def write_summary(path, config, bounds, points):
    field = config["field"]
    x_len = field["x_max_m"] - field["x_min_m"]
    y_len = field["y_max_m"] - field["y_min_m"]
    z_len = field["z_max_m"] - field["z_min_m"]
    with path.open("w", encoding="utf-8") as f:
        f.write("Measured field static map\n")
        f.write("=========================\n\n")
        f.write(
            "Field range: x=[{:.3f},{:.3f}] m, y=[{:.3f},{:.3f}] m, z=[{:.3f},{:.3f}] m\n".format(
                field["x_min_m"],
                field["x_max_m"],
                field["y_min_m"],
                field["y_max_m"],
                field["z_min_m"],
                field["z_max_m"],
            )
        )
        f.write("Launch lengths: map_x_length={:.3f}, map_y_length={:.3f}, map_z_length={:.3f}\n".format(x_len, y_len, z_len))
        f.write("Point count: {}\n\n".format(len(points)))
        for box in bounds:
            f.write(
                "{name}: right_front=({rf_x:.3f},{rf_y:.3f}), "
                "x=[{x_min:.3f},{x_max:.3f}], y=[{y_min:.3f},{y_max:.3f}], z=[{z_min:.3f},{z_max:.3f}]\n".format(
                    name=box["name"],
                    rf_x=box["right_front"][0],
                    rf_y=box["right_front"][1],
                    x_min=box["x_min"],
                    x_max=box["x_max"],
                    y_min=box["y_min"],
                    y_max=box["y_max"],
                    z_min=box["z_min"],
                    z_max=box["z_max"],
                )
            )


def render_top_png(path, config, bounds):
    if Image is None:
        return False

    field = config["field"]
    scale = 120
    margin = 90
    width = int(round((field["x_max_m"] - field["x_min_m"]) * scale)) + 2 * margin
    height = int(round((field["y_max_m"] - field["y_min_m"]) * scale)) + 2 * margin
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)

    def world_to_px(x, y):
        px = margin + (x - field["x_min_m"]) * scale
        py = height - margin - (y - field["y_min_m"]) * scale
        return int(round(px)), int(round(py))

    field_rect = (*world_to_px(field["x_min_m"], field["y_max_m"]),
                  *world_to_px(field["x_max_m"], field["y_min_m"]))
    draw.rectangle(field_rect, outline=(20, 20, 20), width=3)

    # Grid at 0.5 m.
    gx = math.ceil(field["x_min_m"] * 2) / 2.0
    while gx <= field["x_max_m"] + 1e-9:
        p0 = world_to_px(gx, field["y_min_m"])
        p1 = world_to_px(gx, field["y_max_m"])
        color = (210, 210, 210) if abs(gx) > 1e-9 else (120, 120, 120)
        draw.line((p0, p1), fill=color, width=2 if abs(gx) < 1e-9 else 1)
        draw.text((p0[0] - 14, height - margin + 8), f"{gx:g}", fill=(60, 60, 60))
        gx += 0.5

    gy = math.ceil(field["y_min_m"] * 2) / 2.0
    while gy <= field["y_max_m"] + 1e-9:
        p0 = world_to_px(field["x_min_m"], gy)
        p1 = world_to_px(field["x_max_m"], gy)
        color = (210, 210, 210) if abs(gy) > 1e-9 else (120, 120, 120)
        draw.line((p0, p1), fill=color, width=2 if abs(gy) < 1e-9 else 1)
        draw.text((10, p0[1] - 8), f"{gy:g}", fill=(60, 60, 60))
        gy += 0.5

    colors = [(224, 87, 71), (69, 137, 209), (79, 170, 91)]
    for idx, box in enumerate(bounds):
        p0 = world_to_px(box["x_min"], box["y_max"])
        p1 = world_to_px(box["x_max"], box["y_min"])
        draw.rectangle((*p0, *p1), fill=colors[idx % len(colors)], outline=(0, 0, 0), width=3)
        rf = world_to_px(*box["right_front"])
        draw.ellipse((rf[0] - 6, rf[1] - 6, rf[0] + 6, rf[1] + 6), fill=(0, 0, 0))
        draw.text((rf[0] + 8, rf[1] - 18), f"{box['name']} RF", fill=(0, 0, 0))
        center = world_to_px((box["x_min"] + box["x_max"]) / 2.0, (box["y_min"] + box["y_max"]) / 2.0)
        draw.text((center[0] - 26, center[1] - 8), box["name"], fill=(255, 255, 255))

    draw.text((margin, 20), "Top view: measured obstacle map (unit: m)", fill=(0, 0, 0))
    draw.text((width - margin - 70, height - margin + 35), "+x", fill=(0, 0, 0))
    draw.text((margin - 55, margin - 25), "+y", fill=(0, 0, 0))
    image.save(path)
    return True


def svg_line(p1, p2, color="#333", width=1.5):
    return f'<line x1="{p1[0]:.1f}" y1="{p1[1]:.1f}" x2="{p2[0]:.1f}" y2="{p2[1]:.1f}" stroke="{color}" stroke-width="{width}"/>'


def render_svg_3d(path, config, bounds):
    field = config["field"]
    width, height = 1100, 760
    cx, cy = 530, 440
    sx, sy, sz = 85, 85, 110

    def project(x, y, z):
        px = cx + (x - y) * sx
        py = cy + (x + y) * sy * 0.42 - z * sz
        return px, py

    def box_faces(box):
        x0, x1 = box["x_min"], box["x_max"]
        y0, y1 = box["y_min"], box["y_max"]
        z0, z1 = box["z_min"], box["z_max"]
        v = {
            "000": project(x0, y0, z0),
            "100": project(x1, y0, z0),
            "110": project(x1, y1, z0),
            "010": project(x0, y1, z0),
            "001": project(x0, y0, z1),
            "101": project(x1, y0, z1),
            "111": project(x1, y1, z1),
            "011": project(x0, y1, z1),
        }
        return v

    items = []
    items.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">')
    items.append('<rect width="100%" height="100%" fill="white"/>')
    items.append('<text x="40" y="40" font-family="Arial" font-size="24" fill="#222">3D check view: measured obstacle map</text>')

    floor = {
        "x_min": field["x_min_m"],
        "x_max": field["x_max_m"],
        "y_min": field["y_min_m"],
        "y_max": field["y_max_m"],
        "z_min": 0,
        "z_max": 0,
    }
    fv = box_faces(floor)
    floor_poly = " ".join(f"{fv[k][0]:.1f},{fv[k][1]:.1f}" for k in ["000", "100", "110", "010"])
    items.append(f'<polygon points="{floor_poly}" fill="#f3f3f3" stroke="#333" stroke-width="2"/>')

    colors = ["#e05747", "#4589d1", "#4faa5b"]
    for idx, box in enumerate(bounds):
        v = box_faces(box)
        top = " ".join(f"{v[k][0]:.1f},{v[k][1]:.1f}" for k in ["001", "101", "111", "011"])
        side_a = " ".join(f"{v[k][0]:.1f},{v[k][1]:.1f}" for k in ["100", "110", "111", "101"])
        side_b = " ".join(f"{v[k][0]:.1f},{v[k][1]:.1f}" for k in ["010", "110", "111", "011"])
        color = colors[idx % len(colors)]
        items.append(f'<polygon points="{side_b}" fill="{color}" opacity="0.58" stroke="#222" stroke-width="2"/>')
        items.append(f'<polygon points="{side_a}" fill="{color}" opacity="0.70" stroke="#222" stroke-width="2"/>')
        items.append(f'<polygon points="{top}" fill="{color}" opacity="0.85" stroke="#222" stroke-width="2"/>')
        rf = project(box["right_front"][0], box["right_front"][1], 0)
        items.append(f'<circle cx="{rf[0]:.1f}" cy="{rf[1]:.1f}" r="6" fill="#111"/>')
        label = project((box["x_min"] + box["x_max"]) / 2.0, (box["y_min"] + box["y_max"]) / 2.0, box["z_max"] + 0.2)
        items.append(f'<text x="{label[0]:.1f}" y="{label[1]:.1f}" font-family="Arial" font-size="18" text-anchor="middle" fill="#111">{box["name"]}</text>')

    origin = project(0, 0, 0)
    x_axis = project(1.0, 0, 0)
    y_axis = project(0, 1.0, 0)
    z_axis = project(0, 0, 1.0)
    items.append(svg_line(origin, x_axis, "#d13f31", 4))
    items.append(svg_line(origin, y_axis, "#2f7fcc", 4))
    items.append(svg_line(origin, z_axis, "#31934b", 4))
    items.append(f'<text x="{x_axis[0] + 8:.1f}" y="{x_axis[1]:.1f}" font-family="Arial" font-size="18" fill="#d13f31">+x</text>')
    items.append(f'<text x="{y_axis[0] + 8:.1f}" y="{y_axis[1]:.1f}" font-family="Arial" font-size="18" fill="#2f7fcc">+y</text>')
    items.append(f'<text x="{z_axis[0] + 8:.1f}" y="{z_axis[1]:.1f}" font-family="Arial" font-size="18" fill="#31934b">+z</text>')
    items.append("</svg>")
    path.write_text("\n".join(items), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="src/real_uav_bridge/maps/real_field_obstacles.json")
    parser.add_argument("--out-dir", default="src/real_uav_bridge/maps/generated")
    args = parser.parse_args()

    config_path = Path(args.config)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    config = json.loads(config_path.read_text(encoding="utf-8"))
    bounds = [obstacle_bounds(obs, config["obstacle_size_m"]) for obs in config["obstacles"]]
    points = generate_points(bounds, float(config["sampling_step_m"]))

    csv_path = out_dir / "real_field_obstacles.csv"
    summary_path = out_dir / "real_field_obstacles_summary.txt"
    top_png_path = out_dir / "real_field_obstacles_top.png"
    svg_path = out_dir / "real_field_obstacles_3d.svg"

    write_csv(csv_path, points, bounds)
    write_summary(summary_path, config, bounds, points)
    render_top_png(top_png_path, config, bounds)
    render_svg_3d(svg_path, config, bounds)

    print(f"wrote {csv_path} ({len(points)} points)")
    print(f"wrote {summary_path}")
    if top_png_path.exists():
        print(f"wrote {top_png_path}")
    print(f"wrote {svg_path}")


if __name__ == "__main__":
    main()
