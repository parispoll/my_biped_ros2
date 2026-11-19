#!/usr/bin/env python3
"""Educational replay reconstructed from measured joints and physics sole corners.

This is not Gazebo screen capture. A rigid fit of the supporting sole locates
measured joint FK in world coordinates. Time is compressed for inspection.
"""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse, json, os
from bisect import bisect_right
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]


def render(folder):
    os.environ["BIPED_ACTUATOR_PROFILE"] = "robotis_p54"
    import xacro
    from my_robot_description.model.roll_geometry import link_transforms
    from my_robot_description.motion.foot_lift import sole_corners
    from my_robot_description.model.roll_stance import JOINT_NAMES

    summary = json.loads((folder / "summary.json").read_text())
    walking = "alternating_walk" in summary
    audit = summary["alternating_walk"] if walking else summary["first_step"]
    start = audit["origin_sim_time"]
    support = (
        summary["motion_plan"]["supporting_foot"]
        if walking
        else audit["supporting_foot"]
    )
    swing = (
        ("right" if support == "left" else "left") if walking else audit["swing_foot"]
    )
    read = lambda name: [
        json.loads(l) for l in (folder / name).read_text().splitlines()
    ]
    # Exclude startup rows; round-off at the exact origin must not select a
    # previous row whose motion has not started yet.
    joints = [r for r in read("joints.jsonl") if r.get("shift") is not None]
    com = read("com.jsonl")
    jt = [r["sim_time"] for r in joints]
    ct = [r["sim_time"] for r in com]
    xml = xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
    ).toxml()
    fontpath = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(fontpath, 16)
    small = ImageFont.truetype(fontpath, 13)
    big = ImageFont.truetype(fontpath, 23)
    sample_times = np.concatenate(
        [
            np.linspace(110, 117, 14, endpoint=False),
            np.linspace(117, 169, 104, endpoint=False),
            np.linspace(169, 241, 72),
        ]
    )
    if walking:
        plan = summary["motion_plan"]
        parts = []
        last = 0.0
        for step in plan["steps"]:
            parts.extend(
                [
                    np.linspace(last, step["lift_start"], 6, endpoint=False),
                    np.linspace(
                        step["lift_start"], step["reload_start"], 24, endpoint=False
                    ),
                    np.linspace(step["reload_start"], step["end"], 8, endpoint=False),
                ]
            )
            last = step["end"]
        sample_times = np.concatenate([*parts, [last]])
    frames = []
    colors = dict(left="#2076b8", right="#e27622")
    initial = None
    for elapsed in sample_times:
        stamp = start + elapsed
        i = max(0, bisect_right(jt, stamp) - 1)
        k = bisect_right(ct, stamp) - 1
        row = joints[i]
        ob = com[k]
        if walking:
            support = row["shift"]["supporting_foot"]
            swing = row["shift"]["swing_foot"]
        if stamp - jt[i] > 0.1 or stamp - ct[k] > 0.1:
            raise ValueError("Replay lacks fresh recorded geometry")
        tf = link_transforms(xml, dict(zip(JOINT_NAMES, row["position"])))
        local = np.array(sole_corners(xml, tf, support))
        world = np.array(ob["feet"][support])
        a = local.mean(axis=0)
        b = world.mean(axis=0)
        u, _, vt = np.linalg.svd((local - a).T @ (world - b))
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        translation = b - rotation @ a
        if np.max(np.abs(local @ rotation.T + translation - world)) > 0.0001:
            raise ValueError("Replay rigid sole fit exceeds 0.1 mm")
        point = (
            lambda name: rotation @ np.array([tf[name][j][3] for j in range(3)])
            + translation
        )
        image = Image.new("RGB", (1000, 620), "#f6f8fb")
        d = ImageDraw.Draw(image)
        d.text(
            (28, 16),
            (
                "ALTERNATING WALK — measured reconstruction"
                if walking
                else "FIRST FORWARD STEP — measured reconstruction"
            ),
            font=big,
            fill="#172638",
        )
        phase = row["shift"]["phase"].replace("_", " ")
        d.text(
            (28, 52),
            f'{"Step "+str(row["shift"]["step_index"])+" / "+str(summary["motion_plan"]["step_count"])+" — " if walking else ""}{phase.upper()}   |   t = {elapsed:.1f} simulated s after readiness',
            font=font,
            fill="#34475b",
        )
        for panel, axis, title in [
            (0, 0, "SIDE VIEW   +X forward →"),
            (1, 1, "FRONT VIEW   +Y →"),
        ]:
            left = 28 + panel * 490
            top = 95
            width = 460
            height = 405
            floor = top + height
            d.rounded_rectangle(
                (left, top, left + width, floor + 12),
                12,
                fill="white",
                outline="#d9e1eb",
            )
            d.text((left + 14, top + 12), title, font=font, fill="#34475b")
            project = lambda p: (
                left
                + width / 2
                + (
                    float(p[axis]) - (ob["com"][0] if walking else 0.02)
                    if axis == 0
                    else float(p[axis])
                )
                * 540,
                floor - float(p[2]) * 540,
            )
            d.line(
                (left + 12, floor, left + width - 12, floor), fill="#8597ab", width=2
            )
            for side in ("left", "right"):
                points = [
                    point(side + "_thigh"),
                    point(side + "_shin"),
                    point(side + "_foot"),
                ]
                for p, q in zip(points, points[1:]):
                    d.line((*project(p), *project(q)), fill=colors[side], width=12)
                for p in points:
                    x, y = project(p)
                    d.ellipse(
                        (x - 7, y - 7, x + 7, y + 7),
                        fill=colors[side],
                        outline="white",
                        width=2,
                    )
                corners = np.array(ob["feet"][side])
                normal = np.cross(corners[1] - corners[0], corners[3] - corners[0])
                normal /= np.linalg.norm(normal)
                poly = np.concatenate((corners, corners + 0.045 * normal))
                # Convex hull in the two-dimensional drawing plane.
                from scipy.spatial import ConvexHull

                projected = np.array([project(p) for p in poly])
                hull = ConvexHull(projected)
                d.polygon(
                    [tuple(projected[j]) for j in hull.vertices], fill=colors[side]
                )
            hip = point("base_link")
            body = [
                hip + rotation @ np.array([x, y, z])
                for x, y, z in [
                    (-0.08, -0.1, 0),
                    (0.08, -0.1, 0),
                    (0.08, 0.1, 0),
                    (-0.08, 0.1, 0),
                    (-0.08, -0.1, 0.12),
                    (0.08, -0.1, 0.12),
                    (0.08, 0.1, 0.12),
                    (-0.08, 0.1, 0.12),
                ]
            ]
            from scipy.spatial import ConvexHull

            body2 = np.array([project(p) for p in body])
            hull = ConvexHull(body2)
            d.polygon(
                [tuple(body2[j]) for j in hull.vertices],
                fill="#bac9d9",
                outline="#758aa2",
            )
            x, y = project(ob["com"])
            d.ellipse(
                (x - 6, y - 6, x + 6, y + 6), fill="#00a8b5", outline="white", width=2
            )
            gx, gy = project([*ob["com"][:2], 0])
            d.line((x, y, gx, gy), fill="#00a8b5", width=1)
            d.ellipse((gx - 4, gy - 4, gx + 4, gy + 4), fill="#e3b52a")
        foot = np.array(ob["feet"][swing])
        x = foot[:, 0].mean()
        if initial is None:
            initial = (
                {s: sum(c[0] for c in ob["feet"][s]) / 4 for s in ("left", "right")}
                if walking
                else x
            )
        advance = (x - (initial[swing] if walking else initial)) * 1000
        height = foot[:, 2].min() * 1000
        if abs(height) < 0.001:
            height = 0.0
        d.text(
            (28, 532),
            f"{swing.capitalize()} foot: {advance:5.1f} mm forward  |  lowest sole corner: {height:5.1f} mm",
            font=font,
            fill="#172638",
        )
        d.text(
            (28, 561),
            "Blue: left leg   Orange: right leg   Cyan: measured COM   Yellow: ground projection",
            font=small,
            fill="#34475b",
        )
        d.text(
            (28, 586),
            "Reconstructed from recorded joint/sole data; schematic body. Time compressed; not Gazebo video.",
            font=small,
            fill="#53687e",
        )
        frames.append(image.convert("P", palette=Image.Palette.ADAPTIVE, colors=128))
    frames[0].save(
        folder / ("walking-replay.gif" if walking else "step-replay.gif"),
        save_all=True,
        append_images=frames[1:],
        duration=[75] * (len(frames) - 1) + [2000],
        loop=0,
        optimize=False,
    )
    return folder / ("walking-replay.gif" if walking else "step-replay.gif")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    print(render(args.folder.resolve()))
