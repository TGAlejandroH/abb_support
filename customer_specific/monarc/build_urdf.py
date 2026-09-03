"""Emit the MONARC cell URDFs: IRB 4600-20/2.50 + IRBP D600 L2000 D1200.

Reads  bundle/meshes_manifest.json   (written by build_meshes.py)
Writes bundle/monarc_stn1.urdf and bundle/monarc_stn2.urdf

One URDF per station (D4), because the app lists every moving URDF joint that no mechanism
group claims under "Unassigned Joints" (SceneTreeDock).  Both stations and the index axis
existing as live joints in one shared URDF therefore left four axes unassigned in every
template.  Each station's URDF instead FREEZES the index axis and the idle station -- which
is what M2 says we do anyway -- by baking their angles into fixed-joint origins.  The idle
station keeps all its geometry, so it still collides; it just cannot be driven.

That leaves 10 moving joints per template -- rail, six arm, tilt, rotate, tailstock --
against the reference project's 9 (rail, six arm, tilt, rotate).

The rail is a zero-range prismatic joint, `lower="0.0" upper="0.0"`, mirroring
Massiv_frame.tgs exactly.  MONARC has no rail hardware; the joint exists because the Weld
Planner's projects are shaped that way, and a single preset at 0 keeps the structure
parallel without ever moving the robot.

Provenance for every number is in tgs_urdf_from_robotstudio_v1.md:

* Robot joint origins   -- inverse(MechanismLinkInstance.CorrectionTransform) from the
  RobotStudio station, cross-checked against the station's own DH block and the IRB 4600
  datasheet nominals (495 / 175 / 1095 / 1230.5 / 175, tool0 at wrist + 85).
* Positioner origins    -- the same derivation from IRBP_D600...rslib / PIM.xml.
* base -> positioner_base -- FITTED to the four calibrated `ARM_TYPE.rot_axis_pose_*`
  entries in the cell's MOC.cfg, which give each station axis directly in the robot frame.
  Residuals after the fit: 0.25 / 3.39 / 1.04 / 4.16 mm and <= 0.27 deg.
* Joint limits          -- MOC.cfg working ranges (see joint_limits.md).
* tcp_torch             -- from the Binzel CAD: nozzle end at (0.0166, 0, 0.4239) with the
  neck 45.12 deg off tool0 +Z, plus 15 mm of stick-out.  MUST be reconciled with the
  controller's tTG_Weld before welding.

One link per CAD part, because the viewer resolves a single mesh and a single colour per
link name -- multi-part links cannot otherwise show their real colours.
"""

from __future__ import annotations

import json
import math as m
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom

HERE = Path(__file__).resolve().parent
BUNDLE = HERE / "bundle"
MESH_DIR = "meshes"


def urdf_path(station):
    return BUNDLE / f"monarc_stn{station}.urdf"

PI = 3.14159265358979
HALF_PI = 1.57079632679490

# base -> positioner_base, fitted to MOC.cfg's calibrated station axis poses
POSITIONER_PLACEMENT = ((1.222672, 0.794993, -0.576539), (0.000587, 0.147569, 0.002287))

# tool0 -> tcp_torch: Binzel nozzle end + 15 mm stick-out, neck 45 deg off +Z
TCP_TORCH = ((0.027200, 0.0, 0.434400), (0.0, 0.785398163, 0.0))

VEL = 3.141593  # placeholder, as in the reference FANUC bundle; per-axis speeds are not
                # in the readable part of MOC.cfg

# name, parent, child, type, xyz, rpy, (lower, upper) or None
JOINTS = [
    # Zero-range rail between `base` and `base_link`, as in Massiv_frame.tgs.  At 0 it is a
    # no-op, so every robot link frame below is unchanged.
    ("linear_rail_joint", "base", "linear_rail", "prismatic", (0, 0, 0), (0, 0, 0), (0.0, 0.0)),
    ("linear_rail_to_base_link", "linear_rail", "base_link", "fixed", (0, 0, 0), (0, 0, 0), None),

    ("joint_1", "base_link", "link_1", "revolute", (0, 0, 0.495), (0, 0, 0), (-PI, PI)),
    ("joint_2", "link_1", "link_2", "revolute", (0.175, 0, 0), (-HALF_PI, -HALF_PI, 0), (-HALF_PI, 2.61799388)),
    ("joint_3", "link_2", "link_3", "revolute", (1.095, 0, 0), (0, 0, 0), (-PI, 1.30899694)),
    ("joint_4", "link_3", "link_4", "revolute", (0.175, 1.2305, 0), (-HALF_PI, 0, 0), (-6.98131701, 6.98131701)),
    ("joint_5", "link_4", "link_5", "revolute", (0, 0, 0), (-HALF_PI, 0, PI), (-2.09439510, 2.09439510)),
    ("joint_6", "link_5", "link_6", "revolute", (0, 0, 0), (HALF_PI, 0, 0), (-6.98131701, 6.98131701)),

    # ABB's tool0 IS the mounting flange, 85 mm past the wrist centre along axis 6
    ("link_6-flange", "link_6", "flange", "fixed", (0, 0, 0.085), (0, 0, 0), None),
    ("flange-tool0", "flange", "tool0", "fixed", (0, 0, 0), (0, 0, 0), None),
    ("tool0-torch", "tool0", "torch", "fixed", (0, 0, 0), (0, 0, 0), None),
    ("tool0-tcp_torch", "tool0", "tcp_torch", "fixed", TCP_TORCH[0], TCP_TORCH[1], None),

    ("base-positioner_base", "base", "positioner_base", "fixed",
     POSITIONER_PLACEMENT[0], POSITIONER_PLACEMENT[1], None),

    # Index / interchange turntable.  Declared here as a real joint so the geometry and
    # limits stay in one place; `frozen_joints()` bakes it to a fixed joint per template.
    ("positioner_index_joint", "positioner_base", "positioner_index", "revolute",
     (1.31, -0.568, 1.25), (0, 0, -HALF_PI), (-0.01745329, 3.15904595)),

    ("stn1_tilt_joint", "positioner_index", "stn1_tilt", "revolute",
     (0.288, -1.31, 0), (HALF_PI, HALF_PI, 0), (-3.15904595, 3.15904595)),
    ("stn1_plate_joint", "stn1_tilt", "stn1_plate", "revolute",
     (0.25, 0, 0), (-HALF_PI, 0, 0), (-20.0, 20.0)),

    ("stn2_tilt_joint", "positioner_index", "stn2_tilt", "revolute",
     (-0.288, 1.309, 0), (-HALF_PI, HALF_PI, 0), (-3.15904595, 3.15904595)),
    ("stn2_plate_joint", "stn2_tilt", "stn2_plate", "revolute",
     (0.25, 0, 0), (-HALF_PI, 0, 0), (-20.0, 20.0)),

    # Tailstock: slides along the rotation axis (+Y in the tilt frame).  Origin sits on the
    # faceplate, so the joint value reads as the distance from the faceplate.  Locked per
    # project and mapped `not_exported` so it never reaches extjoint (M5).
    ("stn1_tailstock_joint", "stn1_tilt", "stn1_tailstock", "prismatic",
     (0.25, -0.856, 0), (0, 0, 0), (-2.5, 2.5)),
    ("stn2_tailstock_joint", "stn2_tilt", "stn2_tailstock", "prismatic",
     (0.25, -0.856, 0), (0, 0, 0), (-2.5, 2.5)),
]

PRISMATIC_AXIS = (0, 1, 0)
REVOLUTE_AXIS = (0, 0, 1)

# links that exist purely as frames, with no geometry of their own
FRAME_ONLY = ["base", "linear_rail", "flange", "tool0", "tcp_torch"]

# joints frozen per template, and the value they are frozen at
NOMINAL_TAILSTOCK = 1.2


def frozen_joints(station):
    idle = 2 if station == 1 else 1
    return {
        "positioner_index_joint": 0.0 if station == 1 else PI,
        f"stn{idle}_tilt_joint": 0.0,
        f"stn{idle}_plate_joint": 0.0,
        f"stn{idle}_tailstock_joint": NOMINAL_TAILSTOCK,
    }


def fmt(values):
    return " ".join(f"{float(v):.9g}" for v in values)


def _rpy_to_matrix(rpy):
    r, p, y = (float(v) for v in rpy)
    cr, sr, cp, sp, cy, sy = (m.cos(r), m.sin(r), m.cos(p), m.sin(p), m.cos(y), m.sin(y))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matrix_to_rpy(R):
    sy = m.hypot(R[0][0], R[1][0])
    if sy > 1e-9:
        return (m.atan2(R[2][1], R[2][2]), m.atan2(-R[2][0], sy), m.atan2(R[1][0], R[0][0]))
    return (m.atan2(-R[1][2], R[1][1]), m.atan2(-R[2][0], sy), 0.0)


def bake(xyz, rpy, jtype, value):
    """Fold a locked joint value into its origin, so the joint can become `fixed`."""
    R = _rpy_to_matrix(rpy)
    if jtype == "prismatic":
        a = PRISMATIC_AXIS
        off = [sum(R[i][k] * a[k] for k in range(3)) * value for i in range(3)]
        return tuple(xyz[i] + off[i] for i in range(3)), tuple(rpy)
    c, s = m.cos(value), m.sin(value)
    Rz = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]      # every revolute axis is local Z
    RR = [[sum(R[i][k] * Rz[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return tuple(xyz), _matrix_to_rpy(RR)


def add_link(root, name, mesh=None, colour=None):
    link = ET.SubElement(root, "link", name=name)
    if mesh is None:
        return link
    for section in ("visual", "collision"):
        node = ET.SubElement(link, section)
        ET.SubElement(node, "origin", xyz="0 0 0", rpy="0 0 0")
        geom = ET.SubElement(node, "geometry")
        ET.SubElement(geom, "mesh", filename=f"{MESH_DIR}/{mesh}")
        if section == "visual" and colour is not None:
            mat = ET.SubElement(node, "material", name=f"{name}_material")
            ET.SubElement(mat, "color", rgba=f"{fmt(colour)} 1.0")
    return link


def build(station):
    frozen = frozen_joints(station)
    manifest = json.loads((BUNDLE / "meshes_manifest.json").read_text())
    parts = manifest["parts"]

    by_link = {}
    for entry in parts:
        by_link.setdefault(entry["link"], []).append(entry)

    root = ET.Element("robot", name="monarc_irb4600_irbp_d600")
    root.append(ET.Comment(
        " MONARC cell: ABB IRB 4600-20/2.50 + IRBP D600 L2000 D1200 (2 stations). "
        "Generated by build_meshes.py + build_urdf.py. Do not hand-edit; "
        "see tgs_urdf_from_robotstudio_v1.md for the provenance of every number. "
    ))

    kinematic_links = [j[2] for j in JOINTS]
    kinematic_links.insert(0, JOINTS[0][1])

    extra_fixed = []
    for name in kinematic_links:
        entries = by_link.get(name, [])
        if name in FRAME_ONLY or not entries:
            add_link(root, name)
        elif len(entries) == 1:
            e = entries[0]
            add_link(root, name, e["file"], e["colour_rgb"])
        else:
            # several CAD parts share this kinematic link: one child link each, so every
            # part keeps its own colour (the viewer reads one mesh + one colour per link)
            add_link(root, name)
            for e in entries:
                child = e["file"][: -len("_m.stl")]
                add_link(root, child, e["file"], e["colour_rgb"])
                extra_fixed.append((f"{name}-{child}", name, child))

    for jname, parent, child, jtype, xyz, rpy, limits in JOINTS:
        if jname in frozen:
            xyz, rpy = bake(xyz, rpy, jtype, frozen[jname])
            jtype, limits = "fixed", None
        joint = ET.SubElement(root, "joint", name=jname, type=jtype)
        ET.SubElement(joint, "origin", xyz=fmt(xyz), rpy=fmt(rpy))
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        if jtype in ("revolute", "prismatic"):
            axis = PRISMATIC_AXIS if jtype == "prismatic" else REVOLUTE_AXIS
            ET.SubElement(joint, "axis", xyz=fmt(axis))
            ET.SubElement(joint, "limit", lower=f"{limits[0]:.9g}", upper=f"{limits[1]:.9g}",
                          effort="0", velocity=f"{VEL:.9g}")

    for jname, parent, child in extra_fixed:
        joint = ET.SubElement(root, "joint", name=jname, type="fixed")
        ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)

    xml = minidom.parseString(ET.tostring(root, "utf-8")).toprettyxml(indent="  ")
    xml = "\n".join(line for line in xml.splitlines() if line.strip())
    out = urdf_path(station)
    out.write_text(xml + "\n", encoding="utf-8")

    n_links = len(root.findall("link"))
    n_joints = len(root.findall("joint"))
    moving = [j.get("name") for j in root.findall("joint") if j.get("type") != "fixed"]
    print(f"wrote {out.relative_to(HERE)}")
    print(f"  {n_links} links, {n_joints} joints, {len(moving)} moving: {', '.join(moving)}")
    print(f"  frozen: {', '.join(f'{k}={v:.4g}' for k, v in frozen.items())}")
    return moving


def main():
    for station in (1, 2):
        build(station)


if __name__ == "__main__":
    main()
