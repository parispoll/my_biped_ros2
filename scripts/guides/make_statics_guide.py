#!/usr/bin/env python3
"""Regenerate the visual standing-load guide from the current robot model."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import math
import xacro
from my_robot_description.model.stance import model_from_urdf
from my_robot_description.model.statics import standing_loads

# The desktop's bundled PDF runtime supplies ReportLab; ROS uses system Python.
try:
    from reportlab.pdfgen import canvas
except ImportError:
    sys.path.append(
        "/home/parispoll/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/lib/python3.12/site-packages"
    )
    from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor, white
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "output/pdf"
OUT.mkdir(parents=True, exist_ok=True)
for name, filename in [("Body", "DejaVuSans.ttf"), ("Bold", "DejaVuSans-Bold.ttf")]:
    pdfmetrics.registerFont(
        TTFont(name, "/usr/share/fonts/truetype/dejavu/" + filename)
    )
model = model_from_urdf(
    xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/my_biped.urdf.xacro")
    ).toxml()
)
r = standing_loads(model, 0.4)
h, k, a = r["hip_knee_ankle_rad"]
height = model.geometry((h, k, a))["expected_pelvis_height"]
points = [
    (0, height),
    (-model.thigh * math.sin(h), height - model.thigh * math.cos(h)),
    (r["pressure_center_x_from_pelvis_m"], model.foot_thickness),
]
bodies = [
    (model.thigh_mass, tuple((p + q) / 2 for p, q in zip(points[0], points[1]))),
    (model.shin_mass, tuple((p + q) / 2 for p, q in zip(points[1], points[2]))),
    (model.foot_mass, (points[2][0], model.foot_thickness / 2)),
]
navy = "#17324D"
blue = "#167D9A"
red = "#C65442"
gold = "#AA7100"
gray = "#647586"
c = canvas.Canvas(str(OUT / "standing-torque-visual-guide.pdf"), pagesize=(595, 842))
c.setTitle("Standing torques - a visual guide to our biped")


def text(x, y, s, size=11, color=navy, bold=False):
    c.setFillColor(HexColor(color))
    c.setFont("Bold" if bold else "Body", size)
    c.drawString(x, y, s)


def lines(x, y, items, size=11, leading=19, color=navy):
    for s in items:
        text(x, y, s, size, color)
        y -= leading


def arrow(x1, y1, x2, y2, color):
    c.setStrokeColor(HexColor(color))
    c.setFillColor(HexColor(color))
    c.setLineWidth(2)
    c.line(x1, y1, x2, y2)
    angle = math.atan2(y2 - y1, x2 - x1)
    p = c.beginPath()
    p.moveTo(x2, y2)
    for delta in (0.45, -0.45):
        p.lineTo(x2 - 8 * math.cos(angle + delta), y2 - 8 * math.sin(angle + delta))
    p.close()
    c.drawPath(p, fill=1, stroke=0)


def header(num, title, subtitle):
    c.setFillColor(HexColor("#F5F8FA"))
    c.rect(0, 0, 595, 842, fill=1, stroke=0)
    text(34, 802, "BIPED LAB  /  UNDERSTANDING THE LOADS", 10, blue, True)
    text(34, 765, title, 23, bold=True)
    text(34, 741, subtitle, 10, gray)
    text(
        34,
        23,
        "Current URDF • knee bend 0.4 rad • ideal static two-foot support",
        8,
        gray,
    )
    text(539, 23, f"{num} / 4", 8, gray)


def diagram(selected=None):
    # True proportions in the sagittal plane; both legs overlap in this view.
    scale = 465
    ox = 162
    oy = 368

    def xy(p):
        return ox + scale * p[0], oy + scale * p[1]

    c.setStrokeColor(HexColor("#BBC8D1"))
    c.setLineWidth(1)
    c.line(51, oy, 286, oy)
    c.setFillColor(HexColor("#DCE6EC"))
    c.setStrokeColor(HexColor(navy))
    c.rect(ox - 32, xy(points[0])[1], 64, 36, fill=1, stroke=1)
    c.setLineWidth(13)
    c.setLineCap(1)
    for p, q in zip(points, points[1:]):
        c.line(*xy(p), *xy(q))
    ax, az = xy(points[2])
    c.setLineWidth(1)
    c.rect(
        ax - model.foot_length * scale / 2,
        oy,
        model.foot_length * scale,
        model.foot_thickness * scale,
        fill=1,
        stroke=1,
    )
    for i, (name, p) in enumerate(zip(("HIP", "KNEE", "ANKLE"), points)):
        x, y = xy(p)
        c.setFillColor(HexColor(gold if selected == i else navy))
        c.circle(x, y, 6, fill=1, stroke=0)
        text(38, y + 8, name, 9, bold=True)
        c.setStrokeColor(HexColor(gray))
        c.line(87, y + 10, x - 9, y + 2)
    for i, (mass, p) in enumerate(bodies):
        if selected is None or i >= selected:
            x, y = xy(p)
            c.setFillColor(HexColor(red))
            c.circle(x, y, 3, fill=1, stroke=0)
            if i == 2:
                c.setStrokeColor(HexColor(red))
                c.setLineWidth(1)
                c.line(x, y, x - 26, y)
                arrow(x - 26, y, x - 26, y - 32, red)
                text(x - 89, y - 22, f"{mass*9.81:.3f} N", 9, red)
            else:
                arrow(x, y, x, y - 32, red)
                text(x + 12, y - 14, f"{mass*9.81:.3f} N", 9, red)
    x = ax
    arrow(x, oy - 44, x, oy, blue)
    text(x + 12, oy - 30, f'{r["vertical_ground_force_per_foot_n"]:.4f} N', 9, blue)
    text(51, oy - 65, "+X forward →", 9, gray)
    text(51, oy - 82, "True proportions; one leg shown.", 8, gray)
    if selected is not None:
        x, y = xy(points[selected])
        text(
            306, 680, "CUT HERE: " + ("HIP", "KNEE", "ANKLE")[selected], 12, gold, True
        )
        lines(
            306,
            657,
            [
                "Keep only the bodies below",
                "this joint in the calculation.",
                "",
                "Red arrows: body weights",
                "Blue arrow: floor force",
                "Gold dot: joint being studied",
            ],
            10,
            19,
        )
        lines(
            306,
            518,
            [
                "All forces here are vertical.",
                "Use HORIZONTAL distances",
                "to calculate their moments.",
                "",
                "Downward weight:",
                "moment = distance × weight",
                "",
                "Upward floor force:",
                "moment = −distance × force",
            ],
            10,
            19,
        )


header(
    1,
    "Why do the knees need torque?",
    "Start with the forces, then calculate their turning effects.",
)
diagram()
lines(
    306,
    681,
    [
        "OUR CURRENT ROBOT",
        f"Total mass: {model.mass:.1f} kg",
        f"Weight: {model.mass*9.81:.3f} N",
        "",
        "Two feet share the load:",
        f"{model.mass*9.81:.3f} ÷ 2 = 69.1605 N",
        "upward beneath each foot.",
    ],
    11,
    21,
)
lines(
    306,
    469,
    [
        "JOINT ANGLES",
        f"Hip: {h:.6f} rad",
        f"Knee: {k:.6f} rad",
        f"Ankle: {a:.6f} rad",
        "Sum = 0: foot stays flat.",
    ],
    10,
    20,
)
text(34, 247, "The rule used on every following page", 15, bold=True)
lines(
    34,
    218,
    [
        "A moment is a force’s turning effect about a joint.",
        "Torque units: N·m. Force units: N. They are different quantities.",
        "For vertical forces: moment = horizontal lever arm × force.",
        "At rest: motor torque + floor moment + weight moments = 0.",
    ],
    11,
    22,
)
text(34, 110, "Assumptions matter", 12, bold=True)
lines(
    34,
    88,
    [
        "Upright pelvis, flat feet, equal support, pressure at foot centres, no friction.",
        "No acceleration or push. This estimates equilibrium, not stability or motor size.",
        "Both legs have the same result. Pelvis weight enters through the floor force.",
    ],
    9,
    17,
)
c.showPage()
for index, name in enumerate(("hip", "knee", "ankle")):
    row = r["per_leg"][name]
    jx = points[index][0]
    fx = points[2][0]
    header(
        index + 2,
        f"{index+1}. The {name}: balance the moments",
        "One leg shown. The same calculation applies to the other leg.",
    )
    diagram(index)
    text(34, 249, "Horizontal lever arms measured FROM this joint", 13, bold=True)
    distances = [
        ("Floor pressure centre", fx - jx, r["vertical_ground_force_per_foot_n"])
    ]
    distances += [
        (label + " centre", p[0] - jx, mass * 9.81)
        for label, (mass, p) in zip(("Thigh", "Shin", "Foot")[index:], bodies[index:])
    ]
    yy = 226
    for label, d, f in distances:
        text(34, yy, f"{label:<24}   {d:+.6f} m   |   {f:.4f} N", 10)
        yy -= 18
    # Compact arithmetic stays in a fixed lower panel on all three pages.
    text(
        34,
        138,
        f'Floor moment = −({fx-jx:+.6f}) × 69.1605 = {row["ground_moment_nm"]:+.6f} N·m',
        10,
        blue,
    )
    terms = " + ".join(
        f"({p[0]-jx:+.6f} × {mass*9.81:.3f})" for mass, p in bodies[index:]
    )
    text(34, 118, "Weights = " + terms, 8, red)
    text(
        34,
        99,
        f'Total weight moment = {row["distal_weight_moment_nm"]:+.6f} N·m',
        10,
        red,
    )
    tau = row["motor_torque_nm"]
    shown = 0 if abs(tau) < 1e-10 else tau
    text(
        34,
        72,
        f'MOTOR = −({row["ground_moment_nm"]:+.6f} {row["distal_weight_moment_nm"]:+.6f}) = {shown:+.6f} N·m',
        13,
        bold=True,
    )
    note = {
        "hip": "Moments cancel in this balanced pose. Zero holding torque does not mean stable.",
        "knee": "Negative torque opposes bending: each knee must supply about 2.52 N·m.",
        "ankle": "Zero lever arms under our pressure-centre assumption. A lean changes this.",
    }[name]
    text(34, 47, note, 9)
    c.showPage()
c.save()
print(OUT / "standing-torque-visual-guide.pdf")
