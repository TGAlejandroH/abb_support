"""Verify the two generated .tgs templates.

    ~/anaconda3/envs/pyoccenv/python.exe verify_tgs.py

Checks the things that would be expensive to discover later:

* NO UNASSIGNED JOINTS -- every moving URDF joint is claimed by the robot group or by a
  mechanism group.  Anything left over shows up in the app under "Unassigned Joints"
  (SceneTreeDock), which is exactly the defect this pass was opened to fix;
* the embedded bundle is byte-identical to the verified one on disk (a .tgs carries its own
  copy, so fixing the bundle on disk does nothing for an existing project);
* both templates embed the same MESHES, and differ only in URDF + cuRobo config;
* the configuration matches the reference project Massiv_frame.tgs where it should:
  a zero-preset rail group, a shared/time_shared positioner, tool rows carrying load
  fields, a CAD frame on the plate link, and no pre-selected mechanism group;
* the ABB external-axis mapping names the slots this cell actually uses (eax_b / eax_c),
  and the tailstock is not exported;
* FK run on the URDF *as extracted from the .tgs* puts the active faceplate where the
  controller's calibrated PLATE axis says it is, and the CAD frame rides both axes.
"""

from __future__ import annotations

import hashlib
import json
import sqlite3
import sys
import tempfile
from pathlib import Path

import numpy as np
from urdfpy import URDF

HERE = Path(__file__).resolve().parent
BUNDLE = HERE / "bundle"
PI = np.pi

failures: list[str] = []
checks = 0


def check(label, ok, detail=""):
    global checks
    checks += 1
    print(f"  {'PASS' if ok else 'FAIL'}  {label}{('   ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quat_abb(q):
    w, x, y, z = np.array(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


MOC_PLATE = {
    1: ((1.37036, 0.800883, 0.409268), (0.535299, 0.53563, -0.459753, 0.463878)),
    2: ((1.37014, 0.79792, 0.41657), (0.536457, 0.537943, -0.460038, 0.459561)),
}

mesh_digests = {}
for n in (1, 2):
    path = BUNDLE / f"MONARC_Station{n}.tgs"
    print(f"\n=== MONARC_Station{n}.tgs ===")
    db = sqlite3.connect(f"file:{path.as_posix()}?mode=ro", uri=True)
    db.row_factory = sqlite3.Row

    meta = {k: v for k, v in db.execute("select key,value from project_meta")}
    check("schema_version 8 / tgs-sqlite",
          meta.get("schema_version") == "8" and meta.get("format") == "tgs-sqlite")
    check("bundle mode embedded", meta.get("robot_bundle_mode") == "embedded")
    check(f"work_zone is the station number, as a text integer in 1..10",
          meta.get("work_zone") == str(n), repr(meta.get("work_zone")))
    check("points at this station's URDF and yaml",
          meta["robot_urdf_bundle_path"] == f"monarc_stn{n}.urdf"
          and meta["robot_yaml_bundle_path"] == f"monarc_torch_stn{n}.yaml",
          f'{meta["robot_urdf_bundle_path"]} / {meta["robot_yaml_bundle_path"]}')

    files = {r["path"]: (r["content_blob"], r["sha256"])
             for r in db.execute("select path,content_blob,sha256 from robot_bundle_files")}
    check("25 bundle files", len(files) == 25, str(len(files)))
    bad = [p for p, (blob, sha) in files.items() if hashlib.sha256(blob).hexdigest() != sha]
    check("every sha256 matches its blob", not bad, str(bad))
    mismatched = [p for p, (blob, _) in files.items()
                  if not (BUNDLE / p).is_file() or (BUNDLE / p).read_bytes() != blob]
    check("embedded bundle == verified on-disk bundle", not mismatched, str(mismatched[:3]))
    mesh_digests[n] = {p: sha for p, (_, sha) in files.items() if p.startswith("meshes/")}

    rg = db.execute("select * from robot_groups").fetchone()
    check("robot group base/tcp/brand",
          rg["base_link"] == "base" and rg["tcp_link"] == "link_6" and rg["robot_brand"] == "abb",
          f'{rg["base_link"]}/{rg["tcp_link"]}/{rg["robot_brand"]}')
    arm = json.loads(rg["arm_joint_json"])
    check("6 arm joints", len(arm) == 6)
    check("default mechanism group is the rail",
          json.loads(rg["default_mechanism_groups_json"]) == ["Rail 1"],
          rg["default_mechanism_groups_json"])

    groups = {r["name"]: dict(r) for r in db.execute(
        "select nn.name, g.* from external_axis_groups g join nodes nn on nn.id=g.node_id")}
    check("three mechanism groups: rail, station, tailstock",
          set(groups) == {"Rail 1", f"Station {n}", "Tailstock"}, str(sorted(groups)))

    # The weld editor takes the positioner's DISTAL joint as the coordinated work axis and
    # needs it to have ANGULAR travel; a prismatic one reports None and greys out
    # "Synchronous (coordinated) motion".  This reproduces that gate.
    ts = groups["Tailstock"]
    check("tailstock is its own custom group, out of the positioner",
          ts["kind"] == "custom" and json.loads(ts["joint_json"]) == [f"stn{n}_tailstock_joint"],
          f'kind={ts["kind"]} joints={ts["joint_json"]}')

    railg = groups["Rail 1"]
    check("rail group kind/ownership", railg["kind"] == "rail" and railg["ownership_mode"] == "exclusive")
    rax = json.loads(railg["metadata_json"])["export_v1"]["brands"]["abb"]["axes"][0]
    check("rail has exactly one preset, at 0", rax["index_values"] == ["0"], str(rax["index_values"]))

    eg = groups[f"Station {n}"]
    joints = json.loads(eg["joint_json"])
    check("positioner is exactly two axes, tilt then rotate",
          joints == [f"stn{n}_tilt_joint", f"stn{n}_plate_joint"], str(joints))
    check("positioner ee link", eg["positioner_ee_link"] == f"stn{n}_plate", eg["positioner_ee_link"])
    check("positioner is shared / time_shared",
          eg["kind"] == "positioner" and eg["shared"] == 1 and eg["ownership_mode"] == "time_shared",
          f'kind={eg["kind"]} shared={eg["shared"]} {eg["ownership_mode"]}')

    axes = json.loads(eg["metadata_json"])["export_v1"]["brands"]["abb"]["axes"]
    slots = {a["joint_name"]: (a["label"], a["export"]) for a in axes}
    check("tilt -> eax_b embedded", slots[f"stn{n}_tilt_joint"] == ("eax_b", "embedded_in_pose"))
    check("plate -> eax_c embedded", slots[f"stn{n}_plate_joint"] == ("eax_c", "embedded_in_pose"))
    ts_axes = json.loads(ts["metadata_json"])["export_v1"]["brands"]["abb"]["axes"]
    check("tailstock not_exported", ts_axes[0]["export"] == "not_exported")

    tools = {r["name"]: json.loads(r["xyz_rpy_json"]) for r in db.execute(
        "select nn.name, t.xyz_rpy_json from tools t join nodes nn on nn.id=t.node_id")}
    check("tool rows carry the load fields the reference has",
          all({"cog_xyz", "inertia_rpy", "inertia_xyz", "mass_kg", "xyz", "rpy"} <= set(v)
              for v in tools.values()), str(sorted(tools)))

    state = json.loads(db.execute("select joint_json from robot_state").fetchone()[0])
    ui = {k: v for k, v in db.execute("select key,value from ui_state")}
    check("active external axis group is blank, as in both references",
          ui["active_external_axis_group_name"] == "", repr(ui["active_external_axis_group_name"]))
    check("active tool", ui["active_tool_name"] == "tcp_torch")

    frames = {r["name"]: dict(r) for r in db.execute(
        "select n.id,n.name,n.parent_id,n.parent_ref_type,n.parent_ref_value,"
        "t.tx,t.ty,t.tz,t.rx,t.ry,t.rz from nodes n "
        "left join transforms t on t.node_id=n.id where n.type='frame'")}
    check("a frame named CAD exists", "CAD" in frames, str(sorted(frames)))
    cad = frames.get("CAD", {})
    check("CAD is parented to this station's plate link",
          cad.get("parent_ref_type") == "urdf_link" and cad.get("parent_ref_value") == f"stn{n}_plate",
          f'{cad.get("parent_ref_type")}:{cad.get("parent_ref_value")}')
    check("a Fixtures frame hangs under CAD",
          frames.get("Fixtures", {}).get("parent_id") == cad.get("id"))

    # --- FK on the URDF as extracted FROM the .tgs -------------------------------------
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        for p, (blob, _) in files.items():
            out = td / p
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_bytes(blob)
        robot = URDF.load(str(td / meta["robot_urdf_bundle_path"]))
        moving = [j.name for j in robot.joints if j.joint_type != "fixed"]

        # THE check this pass was opened for
        claimed = set(arm) | {j for g in groups.values() for j in json.loads(g["joint_json"])}
        unassigned = [j for j in moving if j not in claimed]
        check("no unassigned joints", not unassigned, str(unassigned))
        phantom = [j for j in claimed if j not in moving]
        check("no group claims a joint the URDF does not move", not phantom, str(phantom))
        check("saved state covers exactly the moving joints",
              set(state) == set(moving), f"{len(state)} vs {len(moving)}")

        # Reproduce synchronous_motion_available() for a multi-axis positioner: the work
        # axis is the DISTAL joint, judged by its angular travel (>= 5 deg).  A prismatic
        # joint reports None and greys the checkbox out -- the bug this pass fixed.
        jmap = {j.name: j for j in robot.joints}
        work = joints[-1]
        wj = jmap[work]
        travel = (None if wj.joint_type == "prismatic"
                  else np.degrees(wj.limit.upper - wj.limit.lower))
        check("coordinated motion is available (work axis has angular travel)",
              travel is not None and travel >= 5.0,
              f"work axis {work} ({wj.joint_type}): "
              + ("no angular travel" if travel is None else f"{travel:.0f} deg"))

        cfg = {k: v for k, v in state.items() if k in set(moving)}
        T = robot.link_fk(cfg=cfg, link=f"stn{n}_plate")
        pos, quat = MOC_PLATE[n]
        a_model, a_moc = T[:3, 2], quat_abb(quat)[:, 2]
        if np.dot(a_moc, a_model) < 0:
            a_moc = -a_moc
        ang = np.degrees(np.arccos(np.clip(abs(np.dot(a_moc, a_model)), -1, 1)))
        d = np.array(pos) - T[:3, 3]
        perp = np.linalg.norm(d - np.dot(d, a_model) * a_model) * 1000
        check("active faceplate axis vs MOC.cfg PLATE pose", perp < 6.0 and ang < 0.4,
              f"{perp:.2f} mm, {ang:.2f} deg")

        def cad_world(overrides):
            c = dict(cfg)
            c.update(overrides)
            Tp = robot.link_fk(cfg=c, link=f"stn{n}_plate")
            local = np.eye(4)
            local[:3, 3] = (cad["tx"], cad["ty"], cad["tz"])
            ca, sa = np.cos(cad["rz"]), np.sin(cad["rz"])
            local[:3, :3] = np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]])
            return Tp @ local

        home = cad_world({})
        for axis, joint in (("tilt", f"stn{n}_tilt_joint"), ("rotate/chuck", f"stn{n}_plate_joint")):
            moved = cad_world({joint: 0.30})
            dd = np.linalg.norm(moved[:3, 3] - home[:3, 3]) * 1000
            rr = np.degrees(np.arccos(np.clip(
                (np.trace(moved[:3, :3].T @ home[:3, :3]) - 1) / 2, -1, 1)))
            check(f"CAD frame rides the {axis} axis", dd > 1.0 or rr > 1.0,
                  f"0.30 rad moves it {dd:.0f} mm / {rr:.1f} deg")
    db.close()

print("\n=== both templates ===")
check("identical meshes in both", mesh_digests[1] == mesh_digests[2],
      "differing: " + str([k for k in mesh_digests[1] if mesh_digests[1][k] != mesh_digests[2].get(k)]))

print(f"\n{checks - len(failures)}/{checks} checks passed")
if failures:
    print("FAILED: " + "; ".join(failures))
sys.exit(1 if failures else 0)
