#!/usr/bin/env python3
"""Build the educational PDF from calculate_geometry_limits.py's saved results.

Run with the bundled PDF Python runtime (ReportLab); no robot commands.
"""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import json, math
from reportlab.platypus import (
    SimpleDocTemplate,
    Paragraph,
    Spacer,
    Table,
    TableStyle,
    PageBreak,
    KeepTogether,
)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.graphics.shapes import Drawing, Line, Rect, Circle, String, Polygon

ROOT = Path(__file__).resolve().parents[2]
FOLDER = ROOT / "output/pdf/geometry-calculations"
D = json.loads((FOLDER / "calculations.json").read_text())
C = D["single_support_candidate"]
L = D["lifted_candidate"]
P = D["path"]
PDF = ROOT / "output/pdf/humanoid-geometry-and-current-limits.pdf"
for name, file in [
    ("Body", "DejaVuSans.ttf"),
    ("Bold", "DejaVuSans-Bold.ttf"),
    ("Mono", "DejaVuSansMono.ttf"),
]:
    pdfmetrics.registerFont(TTFont(name, "/usr/share/fonts/truetype/dejavu/" + file))
INK = colors.HexColor("#173047")
TEAL = colors.HexColor("#087f8c")
ORANGE = colors.HexColor("#bc5b2b")
PALE = colors.HexColor("#eaf3f4")
GREY = colors.HexColor("#536572")
styles = getSampleStyleSheet()
styles.add(
    ParagraphStyle(
        name="TitleX",
        fontName="Bold",
        fontSize=26,
        leading=32,
        textColor=INK,
        spaceAfter=14,
    )
)
styles.add(
    ParagraphStyle(
        name="HeadX",
        fontName="Bold",
        fontSize=19,
        leading=25,
        textColor=INK,
        spaceAfter=13,
    )
)
styles.add(
    ParagraphStyle(
        name="SubX",
        fontName="Bold",
        fontSize=11,
        leading=15,
        textColor=TEAL,
        spaceBefore=10,
        spaceAfter=6,
    )
)
styles.add(
    ParagraphStyle(
        name="BodyX",
        fontName="Body",
        fontSize=10,
        leading=15,
        textColor=INK,
        spaceAfter=9,
    )
)
styles.add(
    ParagraphStyle(
        name="SmallX",
        fontName="Body",
        fontSize=8,
        leading=11,
        textColor=GREY,
        spaceAfter=7,
    )
)
styles.add(
    ParagraphStyle(
        name="CellX", fontName="Body", fontSize=8.3, leading=11, textColor=INK
    )
)
styles.add(
    ParagraphStyle(
        name="EqX",
        fontName="Mono",
        fontSize=9,
        leading=14,
        textColor=INK,
        backColor=PALE,
        borderPadding=8,
        spaceBefore=5,
        spaceAfter=12,
    )
)
story = []


def para(s, style="BodyX"):
    return Paragraph(s, styles[style])


def add(s, style="BodyX"):
    story.append(para(s, style))


def eq(s):
    add(s.replace("\n", "<br/>"), "EqX")


def title(n, s):
    if story:
        story.append(PageBreak())
    add(f"{n:02d} / {s}", "HeadX")


def table(rows, widths):
    cells = [[para(str(c), "CellX") for c in row] for row in rows]
    t = Table(cells, colWidths=widths, hAlign="LEFT", repeatRows=1)
    t.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), PALE),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("TOPPADDING", (0, 0), (-1, -1), 7),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
                ("LINEBELOW", (0, 0), (-1, 0), 1, TEAL),
                ("LINEBELOW", (0, 1), (-1, -1), 0.3, colors.HexColor("#dce5e8")),
            ]
        )
    )
    story.append(t)
    story.append(Spacer(1, 9))


def fmt(x, n=3):
    return f"{x:.{n}f}"


def deg(x):
    return x * 180 / math.pi


def label(d, x, y, s, size=8, color=INK):
    d.add(String(x, y, s, fontName="Body", fontSize=size, fillColor=color))


def robot_sketch():
    d = Drawing(490, 205)
    # Front projection from the actual saved candidate joint origins.
    for index, (caption, r) in enumerate(
        [
            ("A. Both feet on floor; right load = 0", C),
            ("B. Right foot lifted 20 mm", L),
        ]
    ):
        x0 = 90 + index * 245
        scale = 290
        z0 = 18
        pos = r["positions"]
        shift = r["shift_m"]
        height = D["base"]["expected_pelvis_height_m"] - r["lowering_m"]
        pt = lambda a: (x0 - a[1] * scale, z0 + a[2] * scale)
        d.add(Line(x0 - 68, z0, x0 + 68, z0, strokeColor=GREY))
        d.add(
            Rect(
                x0 - shift * scale - 0.11 * scale,
                z0 + (height - 0.08) * scale,
                0.22 * scale,
                0.16 * scale,
                fillColor=colors.HexColor("#e4a48b"),
                strokeColor=ORANGE,
            )
        )
        for side, col in [("left", TEAL), ("right", colors.HexColor("#3974ae"))]:
            hip = pos[side + "_thigh"]
            knee = pos[side + "_shin"]
            ankle = pos[side + "_foot"]
            a, b, c = pt(hip), pt(knee), pt(ankle)
            for p, q in [(a, b), (b, c)]:
                d.add(Line(*p, *q, strokeWidth=9, strokeColor=col, strokeLineCap=1))
            for p in [a, b, c]:
                d.add(Circle(*p, 3, fillColor=INK, strokeColor=None))
            d.add(
                Rect(
                    c[0] - 0.06 * scale,
                    c[1] - 0.045 * scale,
                    0.12 * scale,
                    0.045 * scale,
                    fillColor=colors.HexColor("#a6b7bf"),
                    strokeColor=INK,
                )
            )
        com = pt(r["com_world_m"])
        d.add(Circle(*com, 4, fillColor=TEAL, strokeColor=colors.white))
        d.add(
            Line(com[0], z0, com[0], com[1], strokeColor=TEAL, strokeDashArray=[3, 3])
        )
        label(d, x0 - 97, 192, caption, 8)
    label(
        d,
        5,
        2,
        "Front projection: left side of drawing is robot left (+Y). Not a hardware packaging drawing.",
        7,
    )
    return d


add("Can our current humanoid<br/>take its first step?", "TitleX")
add("Geometry, static joint loads and current limitations", "SubX")
add("Educational calculation notebook | P54 profile | 1 November 2026", "SmallX")
add(
    "<b>Conclusion:</b> keep this URDF for the next learning milestones. The calculations find static support and small foot-clearance candidates within the current joint and torque limits. They do not prove stable walking or justify commanding a lift today."
)
story.append(robot_sketch())
table(
    [
        ["Quantity", "Calculated candidate"],
        ["Robot mass / weight", f"{D['mass_kg']:.2f} kg / {D['weight_n']:.4f} N"],
        ["Pelvis shift / lowering", "70 mm left / 60 mm down"],
        [
            "Minimum static sole-edge reserve",
            f"{min(C['sole_edge_reserve_m'])*1000:.2f} mm",
        ],
        [
            "Largest static motor utilization",
            f"{C['peak_utilization']*100:.2f}% (left hip roll)",
        ],
        ["Sampled raised-foot example", "20 mm up; 30 mm forward; no commanded motion"],
    ],
    [240, 250],
)
add("Read this distinction first", "SubX")
add(
    "<b>Calculated:</b> geometry and quasi-static torque feasibility in a restricted family. <b>Measured previously:</b> bilateral load transfer and small pushes. <b>Not demonstrated:</b> unloading to zero, single-leg balance, actual foot lift or walking. The supporting-foot pressure distribution is an assumption, not a new measured result.",
    "SmallX",
)

title(2, "Inputs taken from the current model")
add(
    "Coordinates: +X forward, +Y robot left, +Z up. Hip and ankle roll rotate around X; the three pitch joints rotate around Y. Five joints per leg; no hip yaw, waist, arms or articulated toes."
)
table(
    [
        ["Geometry", "Value / meaning"],
        [
            "Thigh and shin lengths",
            "a = b = 0.260 m; maximum hip-to-ankle reach = 0.520 m",
        ],
        ["Hip / nominal ankle spacing", "2 × 0.085 = 0.170 m"],
        ["Foot size (X × Y × Z)", "0.200 × 0.120 × 0.045 m"],
        ["Torso collision box", "0.180 × 0.220 × 0.160 m"],
        ["Leg collision radii", "Thigh 35.00 mm; shin 33.25 mm"],
    ],
    [220, 270],
)
add("Mass accounting - include the heavier motors", "SubX")
eq(
    "M_structure = 7.5 + 2(1.4 + 1.0 + 0.9 + 0.08 + 0.08)\n            = 14.42 kg\nM_actuators = 2(0.855 + 0.740 + 0.855 + 0.740 + 0.740)\n            = 7.86 kg\nM = 14.42 + 7.86 = 22.28 kg\nW = M g = 22.28 x 9.81 = 218.5668 N"
)
table(
    [
        ["Joint, each leg", "Range (rad)", "Range (degrees)", "Torque cap"],
        ["Hip roll", "−0.35 to +0.35", "−20.05 to +20.05", "44.7 Nm"],
        ["Hip pitch", "−1.20 to +1.20", "−68.75 to +68.75", "25.3 Nm"],
        ["Knee pitch", "0 to 2.40", "0 to 137.51", "44.7 Nm"],
        ["Ankle pitch", "−0.80 to +0.80", "−45.84 to +45.84", "25.3 Nm"],
        ["Ankle roll", "−0.35 to +0.35", "−20.05 to +20.05", "25.3 Nm"],
    ],
    [130, 115, 145, 100],
)
add(
    "All P54 joint speed guards are 2.4 rad/s. These are our configured output-shaft limits; electrical, thermal, backlash and torque-speed behavior are not fully modeled. Actuator mass is represented at joint locations with approximate case inertia.",
    "SmallX",
)

title(3, "Reach and inverse kinematics")
add(
    "Start with the usual centered stance: knee = 0.400 rad, hip pitch = −0.229112 rad, ankle pitch = −0.170888 rad. The pitch angles sum to zero, so the sole stays level."
)
eq(
    "x_A = -a sin(q_h) - b sin(q_h + q_k)\nz_A = -a cos(q_h) - b cos(q_h + q_k)\n"
    + f'x_A = {D["foot_x_m"]:.6f} m; z_A = {D["ankle_z_m"]:.6f} m\nh_pelvis = -z_A + 0.045 = {D["base"]["expected_pelvis_height_m"]:.6f} m'
)
add("Why side shifting straightens the knees", "SubX")
eq(
    "r(s,d)^2 = x_A^2 + s^2 + (z_A + d)^2\nr < a + b = 0.520 m  (strictly below straight-leg singularity)\n"
    + f'Extension reserve at s=d=0: 0.520 - r = {D["fixed_height_extension_reserve_m"]*1000:.3f} mm\n'
    + f'At fixed height: |s| < sqrt(0.520^2-x_A^2-z_A^2)\n                         = {D["fixed_height_lateral_reach_m"]*1000:.3f} mm'
)
table(
    [
        ["Pelvis shift", "Predicted COM Y", "Knee angle", "Right share*"],
        *[
            [
                f"{r['shift_m']*1000:.2f} mm",
                f"{r['com_y_m']*1000:.2f} mm",
                f"{r['knee_rad']:.4f} rad",
                f"{r['center_pressure_right_share']*100:.2f}%",
            ]
            for r in D["fixed_height_scan"]
        ],
    ],
    [125, 125, 125, 115],
)
add(
    "*Shares in this table assume vertical reactions at the two sole centers. The last row is almost singular and is not a command target. Reaching zero right load with this assumption would need COM Y = 85 mm; the fixed-height reach boundary only gives about 66.3 mm.",
    "SmallX",
)
add("Solve the bent-leg angles for a candidate", "SubX")
eq(
    "q_roll = atan2(-s, -(z_A+d))\nz_sag = -sqrt(s^2 + (z_A+d)^2)\nq_k = acos((x_A^2+z_sag^2-a^2-b^2)/(2ab))\nq_h = atan2(-x_A,-z_sag)-atan2(b sin(q_k),a+b cos(q_k))\nq_ankle_pitch = -q_h-q_k; q_ankle_roll = -q_roll"
)
add(
    "Lowering d makes the hip-to-ankle vector shorter, allowing more knee bend and lateral reach. Lowering is a useful design variable, not a universal requirement: finite-foot pressure positions allow some static single-support solutions without it.",
    "SmallX",
)

title(4, "Support polygon, COM and foot loads")
add(
    "The center of mass is mass-weighted, not the torso origin. For every link, transform its inertial center into world coordinates first."
)
eq(
    "p_COM = sum(m_i p_i) / sum(m_i)\nF_L + F_R = W\nF_L y_L + F_R y_R = W y_COM\ny_L=+0.085 m; y_R=-0.085 m\nf_R = F_R/W = 0.5 - y_COM/0.170  (sole-center assumption)"
)
add(
    "Our 70/30 hold has y_COM = 0.034 m and pelvis shift = 0.052917 m. Thus F_L = 152.99676 N and F_R = 65.57004 N. Pelvis displacement is not COM displacement."
)
diagram = Drawing(490, 130)
scale = 1400
x0 = 245
for center in [-0.085, 0.085]:
    x = x0 - center * scale
    diagram.add(
        Rect(
            x - 0.06 * scale,
            35,
            0.12 * scale,
            42,
            fillColor=colors.HexColor("#dce5e8"),
            strokeColor=INK,
        )
    )
    diagram.add(
        Rect(x - 0.05 * scale, 42, 0.10 * scale, 28, fillColor=PALE, strokeColor=TEAL)
    )
for yy, txt, col in [
    (0, "Centered", GREY),
    (0.034, "70/30 COM", ORANGE),
    (C["com_world_m"][1], "Candidate COM", TEAL),
]:
    x = x0 - yy * scale
    diagram.add(Line(x, 30, x, 105, strokeColor=col, strokeDashArray=[2, 2]))
    label(
        diagram,
        x - 42 if yy else x + 8,
        117 if yy == C["com_world_m"][1] else 96 if yy == 0 else 15,
        txt,
        8,
        col,
    )
label(diagram, 55, 84, "Left support foot (+Y)", 8)
label(diagram, 305, 84, "Right foot (−Y)", 8)
label(
    diagram,
    5,
    0,
    "One-dimensional front projection. Pale inner region reserves 10 mm from each edge.",
    7,
)
story.append(diagram)
add("When the right foot carries no load", "SubX")
eq(
    "F_L = W; F_R = 0\nRequired supporting-foot CoP = horizontal COM projection\nLeft physical sole: 0.025 <= y <= 0.145 m\nWith 10 mm reserve: 0.035 <= y <= 0.135 m\n"
    + f'Candidate y_COM = {C["com_world_m"][1]:.6f} m\n'
    + f'Inner-edge margin = y_COM - 0.025 = {min(C["sole_edge_reserve_m"])*1000:.3f} mm'
)
add(
    "CoP means center of pressure: where the resultant ground force acts. It can move within the sole. At the current 70/30 COM of 34 mm, the projection is only 9 mm inside the left physical edge - 1 mm outside our 10 mm reserve. A direct lift from that hold is not the next command.",
    "SmallX",
)

title(5, "A static single-support candidate")
add(
    "Search: 35 lateral shifts from 35 to 120 mm, 13 lowering values from 0 to 60 mm; upright pelvis, flat soles, no yaw. Each candidate uses the actual expanded P54 URDF. A total of 344 sampled poses meet the 10 mm pressure reserve and 80% static utilization limits."
)
add(
    "The lowest-utilization 10 mm-reserve sample uses 55 mm shift / 60 mm lowering, 46.35% peak utilization and only 10.41 mm edge margin. We instead examine the best sampled candidate with at least <b>20 mm</b> margin: 70 mm shift / 60 mm lowering, 47.66% peak utilization. This is a finite grid, not a global optimum."
)
eq(
    f'h_pelvis = {D["base"]["expected_pelvis_height_m"]:.6f} - 0.060 = {D["base"]["expected_pelvis_height_m"]-.06:.6f} m\nCOM = ({C["com_world_m"][0]:.6f}, {C["com_world_m"][1]:.6f}, {C["com_world_m"][2]:.6f}) m\nLeft CoP offset from foot center =\n  ({C["cop_offset_m"][0]*1000:.3f} mm forward, {C["cop_offset_m"][1]*1000:.3f} mm lateral)'
)
rows = [["Joint", "Target (rad)", "Static Nm", "Cap Nm", "Use"]]
for n, t in C["torques_nm"].items():
    rows.append(
        [
            n.replace("_", " "),
            fmt(C["q"][n], 4),
            fmt(t),
            fmt(D["limits"][n]["effort"], 1),
            f"{100*abs(t)/D['limits'][n]['effort']:.1f}%",
        ]
    )
table(rows, [165, 85, 85, 75, 80])
add(
    "The right sole is still geometrically on the floor in this endpoint calculation, but its reaction is set to zero. That models an impending lift; it is not measured single support. Both legs initially have matching angles, but their torques differ because only the left foot supplies support.",
    "SmallX",
)
add(
    "The 60 mm lowering is at the boundary of the search. Do not interpret it as an optimal squat depth or a necessary redesign. A timed transfer to this pose, pressure tracking and dynamic torque must be evaluated separately.",
    "SmallX",
)

title(6, "Worked torque calculation for each joint")
add(
    "Cut the robot at a joint and consider everything downstream. Sum gravity moments on those links and, for a supporting leg, the ground-force moment. The motor must oppose the sum. All vectors are expressed in world coordinates."
)
eq(
    "tau_j = -a_j dot [ sum((p_i-p_j) cross (0,0,-m_i g))\n                  + (p_CoP-p_j) cross (0,0,F_foot) ]\n       = tau_gravity + tau_ground"
)
add(
    "a_j is the unit joint axis; p_j is its origin. This is static inverse dynamics: acceleration, Coriolis terms and viscous damping torque are zero. Torque signs follow the URDF axes, not an intuitive left/right convention.",
    "SmallX",
)
names = ["hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll"]
for name in names:
    n = "left_" + name
    t = C["terms"][n]
    cap = D["limits"][n]["effort"]
    add("Left " + name.replace("_", " "), "SubX")
    eq(
        f'tau = ({t["gravity_nm"]:+.5f}) + ({t["ground_nm"]:+.5f})\n    = {C["torques_nm"][n]:+.5f} Nm\nutilization = |tau|/{cap:.1f} = {100*abs(C["torques_nm"][n])/cap:.2f}%'
    )
add(
    "For a roll axis aligned with +X, the vertical-force contribution is −F × lateral lever arm. Example: left ankle roll has COM/CoP 39.940 mm toward the inner edge, giving −218.5668 × (−0.039940) = +8.72949 Nm.",
    "SmallX",
)
add(
    "The ankle pitch axis is rolled with the leg. Its ground contribution uses the full dot/cross product, rather than assuming it remains exactly world +Y. This matters when the leg rolls.",
    "SmallX",
)

title(7, "The lifted leg and a geometric first swing")
add(
    "Keep the candidate pelvis and left sole fixed. Raise the right ankle 20 mm, move it 30 mm forward while raised, then lower it. Flat sole orientation means ankle lift equals sole clearance. No simulation command is sent."
)
eq(
    "Right target during raised-forward hold:\nx = x_A + 0.030 m\ny = -0.070 m relative to its own hip\nz = z_A + 0.060 + 0.020 m\nApply the same roll + sagittal IK as on page 3."
)
rows = [["Joint", "Before lift (rad)", "Raised + forward", "Static Nm at lift"]]
for name in names:
    n = "right_" + name
    rows.append(
        [
            name.replace("_", " "),
            fmt(C["q"][n], 4),
            fmt(L["q"][n], 4),
            fmt(L["torques_nm"][n]),
        ]
    )
table(rows, [155, 110, 115, 110])
add(
    "With zero right reaction, the right motor torques are gravity-only. At the initial zero-load endpoint, hip roll = −2.72549 Nm, hip pitch = −3.07791 Nm, knee = +3.39468 Nm, and both ankle torques are zero in this ideal placement. At the raised-forward pose, right hip pitch rises in magnitude to 4.51402 Nm.",
    "SmallX",
)
add("Sampled path check", "SubX")
table(
    [
        ["Check across 301 samples", "Result"],
        ["Peak static motor utilization", f"{P['max_static_utilization']*100:.2f}%"],
        ["Smallest sole-edge reserve", f"{P['minimum_edge_reserve_m']*1000:.2f} mm"],
        [
            "Smallest joint-limit margin",
            f"{P['minimum_joint_limit_margin_rad']:.4f} rad ({deg(P['minimum_joint_limit_margin_rad']):.2f} degrees)",
        ],
        [
            "Peak estimated joint speed, 3 s path",
            f"{max(P['max_joint_speeds_rad_s'].values()):.3f} rad/s vs 2.4 rad/s guard",
        ],
    ],
    [270, 220],
)
eq(
    "For each of three 1 s segments: u = local time / 1 s\nS(u) = 10u^3 - 15u^4 + 6u^5\nUse S to lift, advance, then lower; endpoints have zero\ncommanded Cartesian velocity and acceleration."
)
add(
    "Speeds use finite differences of sampled IK. Torque numbers remain static even though the example has a duration: inertial torques are omitted. The foot remains 50 mm laterally clear of the other foot, so 30 mm forward travel does not require the feet to pass each other. These samples do not certify touchdown, continuous collision freedom or balance during motion.",
    "SmallX",
)
add(
    "The existing slow lateral audit is stricter than the motor speed guard: 0.05 rad/s. This 3 s illustration exceeds that audit limit. Uniformly stretching it to about 16.7 s would bring the sampled geometric speed to 0.05 rad/s; tracking and dynamics would still need verification.",
    "SmallX",
)

title(8, "Collision and robustness: what remains unknown")
add(
    "The current simple collision shapes allow useful partial checks. They do not represent a buildable motor installation."
)
table(
    [
        ["Calculation / check", "Result and limit"],
        [
            "Lateral foot-to-foot gap",
            "170 − 120 = 50 mm throughout this flat-foot path. A positive Y gap separates the boxes regardless of their X overlap.",
        ],
        [
            "Cross-leg thigh/shin clearance",
            f"Minimum {P['minimum_cross_leg_capsule_clearance_m']*1000:.2f} mm using enclosing capsules across all four left/right cylinder pairs and 301 samples. This is a conservative sampled separation check for those pairs only.",
        ],
        [
            "Torso, same-leg joints and motor cases",
            "Not certified. The simplified chain joins shapes at common axes; joint carriers have no collision geometry. Actuator case masses are included without faithful case collision shapes.",
        ],
        [
            "Continuous motion and ground clearance",
            "301 samples are not a continuous-time proof. The example sole clears the floor by 20 mm at its raised hold; lift and landing intentionally pass through zero clearance.",
        ],
    ],
    [180, 310],
)
add("A moment reserve is not a tested push limit", "SubX")
eq(
    f'Nearest static edge reserve = {min(C["sole_edge_reserve_m"]):.6f} m\nRemaining tipping moment = W x reserve\n                        = {D["weight_n"]*min(C["sole_edge_reserve_m"]):.3f} Nm\nFor a steady lateral force at pelvis height only:\nF_edge = moment / h_pelvis = {D["weight_n"]*min(C["sole_edge_reserve_m"])/(D["base"]["expected_pelvis_height_m"]-.06):.2f} N'
)
add(
    "That last number assumes the COM stays fixed, no horizontal acceleration, a flat rigid sole and a pressure resultant that can move to the edge. It is <b>not</b> a permissible pulse force or a recovery guarantee. A dynamic push changes velocity, COM position and required pressure. Do not change the panel limits from this formula.",
    "SmallX",
)
add("What the geometry still simplifies", "SubX")
add(
    "No hip yaw limits foot-heading control. No waist or arms provide extra mass-shift options. The torso COM is only 20 mm above its frame; its appearance is not a human-like upper-body mass model. Uniform link/case inertias are approximations. Keep these assumptions visible when interpreting results."
)
add("Current software limits also matter", "SubX")
add(
    "The current lateral experiment audits body height within 5 mm of ordinary standing and requires both feet grounded. It also limits measured joint speed to 0.05 rad/s and each foot share to at least 25%. A 60 mm lowering or an airborne foot cannot pass those existing checks. Develop a separately defined, phase-aware lowering/unloading experiment; preserve the old bilateral checks rather than simply relaxing them.",
    "SmallX",
)

title(9, "Development decision and reproducible evidence")
add(
    "<b>No geometry rebuild is justified by these calculations alone.</b> We found a reasonable static candidate and a small geometric swing with torque headroom. The next issue is turning that candidate into a measured, controlled transfer, not adding joints immediately."
)
table(
    [
        ["Stage", "What must be established before advancing"],
        [
            "1. Bilateral lowered transfer",
            "Reach the new lowered posture slowly with both feet loaded; verify joint tracking, COM, actual loads, pressure position and torque composition.",
        ],
        [
            "2. Contact-transition support",
            "Current load guard requires each foot to retain at least 25% load. Empty contact-wrench arrays are currently invalid data. Add explicit grounded / unloading / airborne states before a foot can lift.",
        ],
        [
            "3. Pressure / balance control",
            "The current planner uses sole-center reactions. This candidate needs a CoP about 39.94 mm inward from the supporting-foot center. Implement and validate suitable measured pressure/balance control; static solvability does not make it happen.",
        ],
        [
            "4. Near-zero load, then tiny lift",
            "Verify support margin and measured unloading first, then attempt a small lift with explicit stop/recovery behavior. Repeat on both sides.",
        ],
        [
            "5. Placement and repeated steps",
            "Verify swing dynamics, touchdown, contact switching and robustness before claiming walking.",
        ],
    ],
    [145, 345],
)
add("Calculation verification", "SubX")
v = D["verification"]
add(
    f"Static torques agree with the existing pressure optimizer within {v['single_support_optimizer_torque_agreement_nm']:.2e} Nm. Independent central differences of gravitational potential agree with gravity torques within {v['gravity_potential_difference_max_error_nm']:.2e} Nm. Sampled joint, pressure, torque and cross-leg checks passed. These numerical checks do not replace Gazebo validation.",
    "SmallX",
)
add("Measured evidence kept separate", "SubX")
add(
    "The earlier +1 N and −1 N / 0.2 s trials passed bilateral recovery audits with the left foot loaded. They do not validate this deeper posture or single support. The first negative-pulse trial failed recording coverage and remains failed. No live robot geometry, gains, guard or server configuration was changed for this PDF.",
    "SmallX",
)
add("Reproduce the calculations", "SubX")
eq(
    "source /opt/ros/jazzy/setup.bash\npython3 scripts/planning/calculate_geometry_limits.py"
)

title(10, "Sources, assumptions and how to read the numbers")
add(
    "All dimensions and limits come from local project sources, not from a generic humanoid. Paths below are relative to the workspace root. The PDF uses the P54 profile explicitly; the legacy profile would produce different mass and torque results.",
    "SmallX",
)
table(
    [
        ["Local source", "Role"],
        [
            "src/my_robot_description/urdf/components/properties.xacro",
            "Lengths, body dimensions and original masses",
        ],
        [
            "src/my_robot_description/urdf/components/leg_roll.xacro",
            "Joint axes/limits; collision shapes; link placement",
        ],
        [
            "src/my_robot_description/urdf/components/base.xacro",
            "Torso mass and inertial center",
        ],
        [
            "src/my_robot_description/urdf/components/roll_actuators.xacro",
            "Actuator mass and approximate composite inertia",
        ],
        [
            "src/my_robot_description/config/roll_actuators.yaml",
            "P54 caps, speed guards and actuator masses",
        ],
        [
            "src/my_robot_description/my_robot_description/model/roll_geometry.py",
            "URDF forward kinematics and mass-weighted COM",
        ],
        [
            "src/my_robot_description/my_robot_description/studies/support_feasibility.py",
            "Existing static pressure/torque optimizer used for cross-check",
        ],
        [
            "scripts/planning/calculate_geometry_limits.py",
            "New offline search, explicit lift, gravity and clearance checks",
        ],
        [
            "output/pdf/geometry-calculations/calculations.json",
            "Full per-link inputs, per-joint moments, sampled-path results and source hashes",
        ],
        [
            "output/pdf/geometry-calculations/expanded-p54.urdf",
            "Exact expanded geometry used for this report",
        ],
    ],
    [295, 195],
)
add("Conventions and scope", "SubX")
add(
    "Metres, kilograms, seconds, newtons and newton-metres are used throughout. Rad denotes radians; degrees = rad × 180/pi. A positive torque is about the named URDF axis. Edge reserve is the shortest distance of COM projection to a supporting sole edge in this flat-foot static calculation. Torque utilization is |requested static torque| / configured cap.",
    "SmallX",
)
add(
    "The single-support search uses upright pelvis and no yaw, with both foot frames fixed until the explicit right-foot lift. It assumes vertical ground reactions and no contact yaw moment. Friction, pushes, inertia, control delay, thermal behavior, manufacturing tolerances and full self-collision are not certified. The candidate is at the lowering search boundary. All reported optima are sampled-family results.",
    "SmallX",
)
add("Educational takeaway", "SubX")
add(
    "<b>Reachability asks whether the joints can make a pose. Static feasibility asks whether contact forces and motor torques can hold it. A controller must still create and stabilize those forces while the robot moves.</b> Our current model passes the first two limited offline checks; the third is the next learning task."
)


def footer(canvas, doc):
    canvas.setStrokeColor(TEAL)
    canvas.line(43, 803, 552, 803)
    canvas.setFont("Body", 8)
    canvas.setFillColor(GREY)
    canvas.drawString(43, 814, "HUMANOID LAB  /  GEOMETRY AUDIT  /  P54")
    canvas.drawString(
        43, 25, "Offline calculation, not a walking certification | 1 November 2026"
    )
    canvas.drawRightString(552, 25, str(doc.page))


SimpleDocTemplate(
    str(PDF),
    pagesize=(595.276, 841.89),
    rightMargin=48,
    leftMargin=48,
    topMargin=51,
    bottomMargin=47,
    title="Humanoid geometry and current limits",
    author="Paris - Humanoid learning project",
).build(story, onFirstPage=footer, onLaterPages=footer)
print(PDF)
