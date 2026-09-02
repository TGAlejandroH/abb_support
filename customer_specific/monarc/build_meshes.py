"""Convert the RobotStudio OBJ exports into Weld-Planner-ready STL meshes.

Inputs   D:\\ABB\\CAD Models\\{IRB4600...,IRBP_D600...}\\OBJ\\**.obj   (+ .mtl for colour)
Outputs  customer_specific/monarc/bundle/meshes/*_m.stl  + meshes_manifest.json

Why each transform is what it is -- see tgs_urdf_from_robotstudio_v1.md:

* RobotStudio's OBJ exporter writes **Y-up**: obj(x, y, z) = model(x, z, -y).
  Undo with model = (ox, -oz, oy).  Verified by fitting the Headstock's axis of
  revolution: 1.24 deg from the expected plate axis undone, 89.54 deg raw.
* Robot link parts have identity placement inside their link, so part frame == robot
  model frame.  Positioner parts are authored in their own frames and placed by the
  .rslib ComponentInstance transforms, which are reproduced below.
* Meshes are emitted in **their own link's frame** so the URDF visual/collision origin
  is identity -- the same pattern the known-good FANUC bundle uses for its positioner.
* Metres, and named `*_m.stl`: that suffix is the only unit hint the app honours
  deterministically (CustomSimulatorDock._stl_length_scale_hint).  The byte-sniffing
  fallback assumes >2.0 m means millimetres, which this cell's bed would trip.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import open3d as o3d

CAD_ROOT = Path(r"D:\ABB\CAD Models")
ROBOT_OBJ = CAD_ROOT / "IRB4600_20_250_C_01_3" / "OBJ"
POS_OBJ = CAD_ROOT / "IRBP_D600_D1200-L2000_M2009_REV1_01" / "OBJ"
OUT_DIR = Path(__file__).resolve().parent / "bundle" / "meshes"

# obj(ox, oy, oz) -> model(ox, -oz, oy)
YUP_INV = np.array(
    [[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
)


def frame(x, y, z, t):
    """Build a homogeneous transform from basis vectors expressed in the parent frame."""
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, t
    return T


def rs_transform(row_x, row_y, row_z, row_t):
    """RobotStudio row-vector 4x4 -> column-vector homogeneous transform."""
    T = np.eye(4)
    T[:3, :3] = np.array([row_x, row_y, row_z], dtype=float).T
    T[:3, 3] = row_t
    return T


def invert(T):
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


# --- link frames, expressed in each machine's own model frame ------------------------
# derived as inverse(MechanismLinkInstance.CorrectionTransform); local Z is the joint axis

ROBOT_LINKS = {
    # `base` is the cell root and `base_link` the robot's own base body; the two frames
    # coincide, so the base casting mesh is authored against base_link.
    "base":      frame((1, 0, 0), (0, 1, 0), (0, 0, 1), (0.0, 0.0, 0.0)),
    "base_link": frame((1, 0, 0), (0, 1, 0), (0, 0, 1), (0.0, 0.0, 0.0)),
    "link_1": frame((1, 0, 0), (0, 1, 0), (0, 0, 1), (0.0, 0.0, 0.495)),
    "link_2": frame((0, 0, 1), (1, 0, 0), (0, 1, 0), (0.175, 0.0, 0.495)),
    "link_3": frame((0, 0, 1), (1, 0, 0), (0, 1, 0), (0.175, 0.0, 1.590)),
    "link_4": frame((0, 0, 1), (0, -1, 0), (1, 0, 0), (1.4055, 0.0, 1.765)),
    "link_5": frame((0, 0, -1), (-1, 0, 0), (0, 1, 0), (1.4055, 0.0, 1.765)),
    "link_6": frame((0, 0, -1), (0, 1, 0), (1, 0, 0), (1.4055, 0.0, 1.765)),
    # the TWeldgun mechanism is parked in the station exactly at tool0 (Ry 90 at
    # 1.4905, 0, 1.765), so the torch's own model frame IS tool0.
    "tool0":  frame((0, 0, -1), (0, 1, 0), (1, 0, 0), (1.4905, 0.0, 1.765)),
}

POSITIONER_LINKS = {
    "positioner_base":  np.eye(4),
    "positioner_index": invert(rs_transform((0, 1, 0), (-1, 0, 0), (0, 0, 1), (-0.568, -1.31, -1.25))),
    "stn1_tilt":        invert(rs_transform((0, 0, -1), (0, -1, 0), (-1, 0, 0), (1.25, -0.856, 0.0))),
    "stn1_plate":       invert(rs_transform((0, 1, 0), (0, 0, -1), (-1, 0, 0), (1.0, 0.0, -0.856))),
    "stn2_tilt":        invert(rs_transform((0, 0, 1), (0, 1, 0), (-1, 0, 0), (1.25, 0.28, -2.619))),
    "stn2_plate":       invert(rs_transform((0, -1, 0), (0, 0, 1), (-1, 0, 0), (1.0, 2.619, 0.28))),
}

# The tailstock slides along the rotation axis, which is +Y in the tilt link's frame and
# passes through (0.25, *, 0) there.  Its link frame is planted at the faceplate, so the
# joint value reads directly as "distance from the faceplate" -- i.e. the tube length.
FACEPLATE_IN_TILT = (0.25, -0.856, 0.0)


def _slide_frame(tilt_frame, y_faceplate):
    T = np.eye(4)
    T[:3, 3] = (FACEPLATE_IN_TILT[0], y_faceplate, FACEPLATE_IN_TILT[2])
    return tilt_frame @ T


POSITIONER_LINKS["stn1_tailstock"] = _slide_frame(POSITIONER_LINKS["stn1_tilt"], -0.856)
POSITIONER_LINKS["stn2_tailstock"] = _slide_frame(POSITIONER_LINKS["stn2_tilt"], -0.856)

# --- parts: (output stem, link, source obj, placement in the machine's model frame) ---

# The torch is its own RobotStudio mechanism, so BinzelTorch.obj is authored in the torch's
# own model frame -- which the station parks exactly at tool0 (Ry 90 at 1.4905, 0, 1.765).
# It must therefore NOT be pushed through inv(tool0); its link frame is identity.
TORCH_LINKS = {"torch": np.eye(4)}
TORCH_PARTS = [
    ("torch", "torch", ROBOT_OBJ / "TORCH/BinzelTorch.obj", np.eye(4)),
]

ROBOT_PARTS = [
    ("abb_base",   "base_link", ROBOT_OBJ / "BASE/Base.obj", np.eye(4)),
    ("abb_link_1", "link_1", ROBOT_OBJ / "LINK1/IRB4600_20kg-250_LINK1_CAD_rev04.obj", np.eye(4)),
    ("abb_link_2", "link_2", ROBOT_OBJ / "LINK2/IRB4600_20kg-250_LINK2_CAD_rev04.obj", np.eye(4)),
    ("abb_link_3", "link_3", ROBOT_OBJ / "LINK3/IRB4600_20kg-250_LINK3_CAD_rev04.obj", np.eye(4)),
    ("abb_link_4", "link_4", ROBOT_OBJ / "LINK4/Link4.obj", np.eye(4)),
    ("abb_link_5", "link_5", ROBOT_OBJ / "LINK5/Link5.obj", np.eye(4)),
    ("abb_link_6", "link_6", ROBOT_OBJ / "LINK6/Link6.obj", np.eye(4)),
]

POSITIONER_PARTS = [
    ("pos_interchange_base", "positioner_base",  POS_OBJ / "BASE/Intch2000.obj",
     rs_transform((0, 1, 0), (-1, 0, 0), (0, 0, 1), (1.31, -0.568, 0.481))),
    ("pos_turntable",        "positioner_index", POS_OBJ / "LINK1/Turntable500D.obj",
     rs_transform((1, 0, 0), (0, 1, 0), (0, 0, 1), (0.165, -0.092, 0.0))),
    ("pos_bearing_stn1",     "positioner_index", POS_OBJ / "LINK1/MTD2000.obj",
     rs_transform((0, 0, 1), (0, 1, 0), (-1, 0, 0), (0.947, -0.856, 1.25))),
    ("pos_bearing_stn2",     "positioner_index", POS_OBJ / "LINK1/MTD2000_2.obj",
     rs_transform((0, 0, 1), (0, -1, 0), (1, 0, 0), (1.673, -0.28, 1.25))),
    ("pos_cover",            "positioner_index", POS_OBJ / "LINK1/Cover600D_D1200_L2000.obj",
     np.eye(4)),

    ("stn1_arm",       "stn1_tilt",  POS_OBJ / "LINK2/Arm500D_2000.obj",
     rs_transform((0, 0, 1), (0, 1, 0), (-1, 0, 0), (0.947, -0.856, 1.25))),
    ("stn1_bed",       "stn1_tilt",  POS_OBJ / "LINK2/Bed.obj",
     rs_transform((1, 0, 0), (0, 1, 0), (0, 0, 1), (0.465, 0.282, 1.16))),
    # MTD750 is the *rotate-axis drive*, not a tailstock: it sits immediately behind the
    # faceplate (y -1.405..-0.975 against the faceplate at -0.856) and is the small sibling
    # of the MTD2000 tilt drives.  There is no tailstock anywhere in the ABB CAD.
    ("stn1_rotate_drive", "stn1_tilt",  POS_OBJ / "LINK2/MTD750.obj",
     rs_transform((-1, 0, 0), (0, 0, -1), (0, -1, 0), (0.0, 0.119, 1.0))),
    ("stn1_headstock", "stn1_plate", POS_OBJ / "LINK3/Headstock.obj",
     rs_transform((-1, 0, 0), (0, 0, -1), (0, -1, 0), (0.0, 0.0, 1.0))),

    ("stn2_arm",       "stn2_tilt",  POS_OBJ / "LINK4/Arm500D_2000.obj",
     rs_transform((0, 0, 1), (0, -1, 0), (1, 0, 0), (1.673, -0.28, 1.25))),
    ("stn2_bed",       "stn2_tilt",  POS_OBJ / "LINK4/Bed.obj",
     rs_transform((-1, 0, 0), (0, -1, 0), (0, 0, 1), (2.155, -1.418, 1.16))),
    ("stn2_rotate_drive", "stn2_tilt",  POS_OBJ / "LINK4/MTD750.obj",
     rs_transform((1, 0, 0), (0, 0, -1), (0, 1, 0), (2.619, -1.254, 1.0))),
    ("stn2_headstock", "stn2_plate", POS_OBJ / "LINK5/Headstock.obj",
     rs_transform((1, 0, 0), (0, 0, -1), (0, 1, 0), (2.619, -1.136, 1.0))),
]

# --- synthesized bodies -------------------------------------------------------------
# The customer holds tubes between the faceplate and a tailstock they slide along the bed
# by hand.  No tailstock exists in the ABB CAD, so we stand in a box until real geometry
# arrives.  Sized to straddle the rotation axis and land on the bed top (z = -0.326 in the
# tilt frame); centred on the axis in X and on the joint origin in Y.
PLACEHOLDER_BOXES = [
    ("stn1_tailstock", "stn1_tailstock", (0.30, 0.25, 0.45), (0.0, 0.0, -0.10), [0.85, 0.45, 0.10]),
    ("stn2_tailstock", "stn2_tailstock", (0.30, 0.25, 0.45), (0.0, 0.0, -0.10), [0.85, 0.45, 0.10]),
]

# triangle budget; anything absent is kept at full resolution
DECIMATE_TO = {
    "stn1_bed": 20000,
    "stn2_bed": 20000,
    "stn1_headstock": 15000,
    "stn2_headstock": 15000,
}


# ABB ships the arm CAD with per-part STEP colours, and LINK3 comes through red -- that is
# the logo/decal colour, not the casting.  The real arm is one off-white, so the whole robot
# is forced to the same grey the positioner arms already use (Arm500D_2000.mtl).
ROBOT_GREY = [0.8431, 0.8431, 0.8431]
COLOUR_OVERRIDE = {
    "base_link": ROBOT_GREY, "link_1": ROBOT_GREY, "link_2": ROBOT_GREY,
    "link_3": ROBOT_GREY, "link_4": ROBOT_GREY, "link_5": ROBOT_GREY,
    "link_6": ROBOT_GREY,
}


def read_mtl_colour(obj_path: Path):
    mtl = obj_path.with_suffix(".mtl")
    if not mtl.is_file():
        return None
    for line in mtl.read_text(errors="ignore").splitlines():
        if line.strip().startswith("Kd "):
            parts = line.split()
            if len(parts) >= 4:
                return [round(float(parts[1]), 4), round(float(parts[2]), 4), round(float(parts[3]), 4)]
    return None


def convert(stem, link, obj_path, placement, link_frames):
    mesh = o3d.io.read_triangle_mesh(str(obj_path))
    if len(mesh.triangles) == 0:
        raise RuntimeError(f"no triangles read from {obj_path}")
    before = len(mesh.triangles)

    # obj -> part frame -> machine model frame -> this link's frame
    mesh.transform(invert(link_frames[link]) @ placement @ YUP_INV)

    target = DECIMATE_TO.get(stem)
    if target is not None and before > target:
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=target)
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.compute_triangle_normals()

    out = OUT_DIR / f"{stem}_m.stl"
    if not o3d.io.write_triangle_mesh(str(out), mesh, write_ascii=False):
        raise RuntimeError(f"failed to write {out}")

    pts = np.asarray(mesh.vertices)
    return {
        "file": out.name,
        "link": link,
        "source": str(obj_path.relative_to(CAD_ROOT)),
        "colour_rgb": COLOUR_OVERRIDE.get(link, read_mtl_colour(obj_path)),
        "triangles_in": before,
        "triangles_out": len(mesh.triangles),
        "bbox_min_m": [round(v, 6) for v in pts.min(axis=0)],
        "bbox_max_m": [round(v, 6) for v in pts.max(axis=0)],
    }


def make_box(stem, link, size, centre, colour):
    mesh = o3d.geometry.TriangleMesh.create_box(*size)
    mesh.translate(np.array(centre) - np.array(size) / 2.0)  # create_box grows from the origin
    mesh.compute_triangle_normals()
    out = OUT_DIR / f"{stem}_m.stl"
    if not o3d.io.write_triangle_mesh(str(out), mesh, write_ascii=False):
        raise RuntimeError(f"failed to write {out}")
    pts = np.asarray(mesh.vertices)
    return {
        "file": out.name,
        "link": link,
        "source": "SYNTHESIZED placeholder box - no tailstock exists in the ABB CAD",
        "placeholder": True,
        "colour_rgb": colour,
        "triangles_in": len(mesh.triangles),
        "triangles_out": len(mesh.triangles),
        "bbox_min_m": [round(v, 6) for v in pts.min(axis=0)],
        "bbox_max_m": [round(v, 6) for v in pts.max(axis=0)],
    }


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    entries = []
    for stem, link, obj_path, placement in ROBOT_PARTS:
        entries.append(convert(stem, link, obj_path, placement, ROBOT_LINKS))
    for stem, link, obj_path, placement in TORCH_PARTS:
        entries.append(convert(stem, link, obj_path, placement, TORCH_LINKS))
    for stem, link, obj_path, placement in POSITIONER_PARTS:
        entries.append(convert(stem, link, obj_path, placement, POSITIONER_LINKS))
    for stem, link, size, centre, colour in PLACEHOLDER_BOXES:
        entries.append(make_box(stem, link, size, centre, colour))

    manifest = {
        "units": "metres",
        "frame": "each mesh is authored in its own URDF link frame; visual/collision origin is identity",
        "naming": "the _m.stl suffix is the app's deterministic metres hint",
        "parts": entries,
    }
    (OUT_DIR.parent / "meshes_manifest.json").write_text(json.dumps(manifest, indent=2))

    total_in = sum(e["triangles_in"] for e in entries)
    total_out = sum(e["triangles_out"] for e in entries)
    width = max(len(e["file"]) for e in entries)
    for e in entries:
        # a small in/out difference is just degenerate-triangle removal, not decimation
        note = "  <- decimated" if e["file"][:-6] in DECIMATE_TO else ""
        print(f'  {e["file"]:<{width}}  {e["link"]:<16} {e["triangles_in"]:>7} -> {e["triangles_out"]:>6} tris{note}')
    print(f"\n  {len(entries)} meshes   {total_in} -> {total_out} triangles")


if __name__ == "__main__":
    main()
