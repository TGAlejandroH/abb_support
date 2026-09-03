"""Emit the two MONARC .tgs templates, one per positioner station (D4).

    ~/anaconda3/envs/pyoccenv/python.exe build_tgs.py

Reads  bundle/monarc_irb4600_irbp_d600.urdf, bundle/monarc_torch.yaml, bundle/meshes/*
       tgs_schema.sql  (schema_version 8, dumped from the shipped two_axis sample)
Writes bundle/MONARC_Station1.tgs, bundle/MONARC_Station2.tgs

A .tgs is a SQLite database.  Both templates embed the SAME bundle -- one cell, one URDF --
and differ only in which station is the active external-axis group and where the index axis
is parked:

    Station 1 -> index 0,  stn1_* joints, ee link stn1_plate
    Station 2 -> index pi, stn2_* joints, ee link stn2_plate

NOT included: a weld_workflow_project.  Its payload carries
`metadata.active_tool_selection`, whose every entry in the shipped catalogue names FANUC
assets (`fanuc_torch.yaml`, `fanuc_stl`, FANUC nozzle STLs).  Authoring one now would bake
FANUC asset names into an ABB template; the weld project should be created in-app once an
ABB tool catalogue exists.  See the open items in tgs_urdf_from_robotstudio_v1.md.
"""

from __future__ import annotations

import json
import sqlite3
import uuid
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
BUNDLE = HERE / "bundle"
SCHEMA = HERE / "tgs_schema.sql"
MESHES = BUNDLE / "meshes"

PI = 3.14159265358979

ROBOT_GROUP = "Robot 1"
RAIL_GROUP = "Rail 1"
TAILSTOCK_GROUP = "Tailstock"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# tool poses are stored relative to the robot group's tcp_link (link_6), NOT tool0 --
# confirmed by reproducing the shipped sample's own numbers through its URDF chain.
# The load fields mirror Massiv_frame.tgs, including its mass_kg of 0.0: real torch load
# data is a Phase 14 prerequisite and there is no honest value to invent here.
LOAD = {"cog_xyz": [0.0, 0.0, 0.0], "inertia_rpy": [0.0, 0.0, 0.0],
        "inertia_xyz": [0.0, 0.0, 0.0], "mass_kg": 0.0}
TOOLS = [
    ("flange", {"xyz": [0.0, 0.0, 0.085], "rpy": [0.0, 0.0, 0.0]}),
    ("torch", {"xyz": [0.0, 0.0, 0.085], "rpy": [0.0, 0.0, 0.0]}),
    ("tcp_torch", {"xyz": [0.0272, 0.0, 0.5194], "rpy": [0.0, 0.785398163, 0.0]}),
]

# MONARC has no rail.  The joint exists at a single preset of 0 because the Weld Planner's
# own projects are shaped that way (Massiv_frame.tgs uses lower=upper=0.0 too), and a
# helper-call mapping with one index value keeps the export structure parallel.
RAIL_METADATA = {
    "export_v1": {
        "motion_role": "helper_only",
        "brands": {
            "abb": {
                "controller_group": "",
                "axes": [{"joint_name": "linear_rail_joint", "label": "E1", "unit": "mm",
                          "export": "helper_call", "index_values": ["0"],
                          "helpers": {"0": "RAIL_0"}}],
            }
        },
    }
}

HOME = {j: 0.0 for j in ARM_JOINTS}
HOME["joint_5"] = -0.7853981633974483   # wrist down, mirrors the reference bundle's home

NOMINAL_TUBE_M = 1.2   # default tailstock stand-off; a per-project number (M5)


def stations():
    for n, index_angle in ((1, 0.0), (2, PI)):
        yield {
            "n": n,
            "name": f"Station {n}",
            "index_angle": index_angle,
            "tilt": f"stn{n}_tilt_joint",
            "plate": f"stn{n}_plate_joint",
            "tailstock": f"stn{n}_tailstock_joint",
            "ee_link": f"stn{n}_plate",
            "mech_unit": f"STN{n}",
            "path": BUNDLE / f"MONARC_Station{n}.tgs",
            "urdf": BUNDLE / f"monarc_stn{n}.urdf",
            "yaml": BUNDLE / f"monarc_torch_stn{n}.yaml",
        }


def export_metadata(st):
    """ABB external-axis mapping (D23) for the two real station axes.

    The cell's own MOC.cfg puts ARM on logical axis 8 (`eax_b`) and PLATE on 9 (`eax_c`)
    -- see joint_limits.md.  Authoring this explicitly matters: with no `abb` block the
    translator derives slots from the mechanism's declared axis order, which would put
    them in eax_a/eax_b and be silently wrong for this cell.
    """
    return {
        "export_v1": {
            "motion_role": "extended_axis",
            "brands": {
                "abb": {
                    "controller_group": st["mech_unit"],
                    "axes": [
                        {"joint_name": st["tilt"], "label": "eax_b", "unit": "deg",
                         "export": "embedded_in_pose"},
                        {"joint_name": st["plate"], "label": "eax_c", "unit": "deg",
                         "export": "embedded_in_pose"},
                    ],
                }
            },
        }
    }


def tailstock_metadata(st):
    """The tailstock gets its OWN `custom` group, deliberately not the positioner's.

    The weld editor takes the positioner's DISTAL joint as the coordinated *work* axis
    (`work_joint_name = joint_names[-1]`) and then asks for its angular travel.  With the
    tailstock last that work axis was a prismatic joint, and
    `positioner_joint_travel_deg` returns None for prismatic by design -- so
    `synchronous_motion_available` was False and "Synchronous (coordinated) motion" was
    greyed out.  Coordinated motion is the whole point for MONARC (D3/M4), so the
    tailstock must not sit in the positioner group.

    `custom` also keeps it out of both the Positioner and Rail combos
    (`_candidate_groups_for_kinds` filters on kind), which is why the stray "Tailst" row
    disappears -- while the joint is still claimed, so it is not "Unassigned".
    """
    return {
        "export_v1": {
            "motion_role": "passive",
            "brands": {
                "abb": {
                    "controller_group": "",
                    "axes": [{"joint_name": st["tailstock"], "label": "", "unit": "mm",
                              "export": "not_exported"}],
                }
            },
        }
    }


def build(st):
    now = datetime.now(timezone.utc).isoformat(timespec="seconds")
    if st["path"].exists():
        st["path"].unlink()
    db = sqlite3.connect(st["path"])
    db.executescript(SCHEMA.read_text())

    def node(ntype, name, parent=None, ref_type="root", ref_value=None, order=0, fixed=1):
        nid = str(uuid.uuid4())
        db.execute(
            "insert into nodes(id,type,name,parent_id,parent_ref_type,parent_ref_value,"
            "order_index,is_fixed,created_at,updated_at) values(?,?,?,?,?,?,?,?,?,?)",
            (nid, ntype, name, parent, ref_type, ref_value, order, fixed, now, now))
        return nid

    # --- embedded bundle ---------------------------------------------------------------
    urdf, yml = st["urdf"], st["yaml"]
    payloads = [(urdf.name, urdf.read_bytes()), (yml.name, yml.read_bytes())]
    payloads += [(f"meshes/{p.name}", p.read_bytes()) for p in sorted(MESHES.glob("*.stl"))]
    import hashlib
    for path, blob in payloads:
        db.execute("insert into robot_bundle_files(path,content_blob,sha256) values(?,?,?)",
                   (path, blob, hashlib.sha256(blob).hexdigest()))

    for key, value in {
        "schema_version": "8",
        "format": "tgs-sqlite",
        "robot_bundle_mode": "embedded",
        "robot_bundle_root": "",
        "robot_urdf_bundle_path": urdf.name,
        "robot_yaml_bundle_path": yml.name,
        "saved_at": now,
        "security_mode": "none",
        # Which work zone of the cell this project belongs to: station 1 -> 1, station 2 -> 2.
        # A text integer in 1..10 (work_zone_stamp_plan_v1.md D1/D6), and D5 says the key is
        # always present -- 1 is exactly what the app writes for a project with no weld
        # project, so station 1 carrying "1" changes nothing.
        #
        # WARNING: this row is a WRITE-ONLY MIRROR (D4).  The source of truth is
        # WeldWorkflowProject.metadata["work_zone"] (D3), and ProjectSerializer re-derives
        # this row from the FIRST weld project on every save.  These templates ship without
        # a weld project (M10), so `project_work_zone(None)` returns the default and the
        # first in-app save of Station 2 will rewrite "2" back to "1".  It sticks only once
        # the weld project exists and is stamped via Tools -> Work Zone Settings.
        "work_zone": str(st["n"]),
    }.items():
        db.execute("insert into project_meta(key,value) values(?,?)", (key, value))

    # --- scene tree ---------------------------------------------------------------------
    node("world_frame", "World Origin")
    rg = node("robot_group", ROBOT_GROUP)
    db.execute(
        "insert into robot_groups(node_id,arm_joint_json,base_link,tcp_link,"
        "default_mechanism_groups_json,robot_brand,output_profile_id,home_poses_json) "
        "values(?,?,?,?,?,?,?,?)",
        (rg, json.dumps(ARM_JOINTS), "base", "link_6", json.dumps([RAIL_GROUP]),
         "abb", "", json.dumps({k: HOME for k in ("capture_home", "process_home", "rest_home")})))

    for order, (tool_name, pose) in enumerate(TOOLS):
        tid = node("tool", tool_name, parent=rg, ref_type="node", order=order)
        db.execute("insert into tools(node_id,robot_group_name,xyz_rpy_json) values(?,?,?)",
                   (tid, ROBOT_GROUP, json.dumps({**LOAD, **pose})))

    # Rail first, matching the reference project's group ordering.
    rail = node("external_axis_group", RAIL_GROUP, order=0)
    db.execute(
        "insert into external_axis_groups(node_id,joint_json,kind,shared,"
        "attached_robot_groups_json,ownership_mode,positioner_base_link,positioner_ee_link,"
        "workpiece_frame_name,metadata_json) values(?,?,?,?,?,?,?,?,?,?)",
        (rail, json.dumps(["linear_rail_joint"]), "rail", 0, json.dumps([ROBOT_GROUP]),
         "exclusive", "", "", "", json.dumps(RAIL_METADATA)))

    eg = node("external_axis_group", st["name"], order=1)
    db.execute(
        "insert into external_axis_groups(node_id,joint_json,kind,shared,"
        "attached_robot_groups_json,ownership_mode,positioner_base_link,positioner_ee_link,"
        "workpiece_frame_name,metadata_json) values(?,?,?,?,?,?,?,?,?,?)",
        (eg, json.dumps([st["tilt"], st["plate"]]), "positioner", 1,
         json.dumps([ROBOT_GROUP]), "time_shared", "base", st["ee_link"], "",
         json.dumps(export_metadata(st))))

    ts = node("external_axis_group", TAILSTOCK_GROUP, order=2)
    db.execute(
        "insert into external_axis_groups(node_id,joint_json,kind,shared,"
        "attached_robot_groups_json,ownership_mode,positioner_base_link,positioner_ee_link,"
        "workpiece_frame_name,metadata_json) values(?,?,?,?,?,?,?,?,?,?)",
        (ts, json.dumps([st["tailstock"]]), "custom", 0, json.dumps([ROBOT_GROUP]),
         "exclusive", "", "", "", json.dumps(tailstock_metadata(st))))

    # "Load CAD" in the Weld Planner ribbon attaches workpieces under the frame named by
    # ROBODK_WORKPIECE_CAD_MODEL_PARENT_FRAME, default literal "CAD".  Parented straight to
    # the station's plate link -- as Massiv_frame.tgs does -- so the workpiece rides BOTH
    # the tilt and the rotate/chuck axis.  The translation is the faceplate offset.
    cid = node("frame", "CAD", ref_type="urdf_link", ref_value=st["ee_link"], order=1, fixed=0)
    db.execute("insert into transforms(node_id,tx,ty,tz,rx,ry,rz,ref_frame_id) "
               "values(?,?,?,?,?,?,?,?)", (cid, 0.0, 0.0, -0.856, 0.0, 0.0, PI, None))

    fx = node("frame", "Fixtures", parent=cid, ref_type="node", order=2, fixed=0)
    db.execute("insert into transforms(node_id,tx,ty,tz,rx,ry,rz,ref_frame_id) "
               "values(?,?,?,?,?,?,?,?)", (fx, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, None))

    # --- saved state ---------------------------------------------------------------------
    # only the joints this template's URDF still moves; the index axis and the idle
    # station are baked into fixed joints, so they are not joints any more
    joints = dict(HOME)
    joints["linear_rail_joint"] = 0.0
    joints[st["tilt"]] = 0.0
    joints[st["plate"]] = 0.0
    joints[st["tailstock"]] = NOMINAL_TUBE_M
    db.execute(
        "insert into robot_state(id,joint_json,tcp_json,rail_value,lock_joints_json,"
        "world_from_base_json,active_robot_group_name,active_external_axis_group_name,"
        "updated_at) values(?,?,?,?,?,?,?,?,?)",
        ("main", json.dumps(joints), None, 0.0, json.dumps({}),
         json.dumps([[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]),
         ROBOT_GROUP, "", now))

    for key, value in {
        "active_robot_group_name": ROBOT_GROUP,
        # blank, as in both reference projects -- no mechanism group is pre-selected
        "active_external_axis_group_name": "",
        "active_tool_name": "tcp_torch",
        "active_tool_name_by_group_json": json.dumps({ROBOT_GROUP: "tcp_torch"}),
        "active_workflow_id": "weld_planner",
        "playback_speed": "2.0",
    }.items():
        db.execute("insert into ui_state(key,value) values(?,?)", (key, value))

    db.commit()
    size = st["path"].stat().st_size
    nfiles = db.execute("select count(*) from robot_bundle_files").fetchone()[0]
    db.close()
    print(f"  wrote {st['path'].name:24} {size/1e6:5.1f} MB   {nfiles} bundle files   "
          f"index={st['index_angle']:.4f} rad   ee={st['ee_link']}")


def main():
    needed = [SCHEMA] + [p for st in stations() for p in (st["urdf"], st["yaml"])]
    for missing in [p for p in needed if not p.is_file()]:
        raise SystemExit(f"missing input: {missing}")
    for st in stations():
        build(st)


if __name__ == "__main__":
    main()
