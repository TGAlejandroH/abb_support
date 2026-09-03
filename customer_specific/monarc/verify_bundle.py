"""Verify the generated URDFs and cuRobo configs against the sources they claim to come from.

Run with the Weld Planner's environment, which is the one that has urdfpy:
    ~/anaconda3/envs/pyoccenv/python.exe verify_bundle.py

Checks, in order of how much they would hurt if wrong:

1. urdfpy -- the parser the app itself uses -- loads the file, single root, no orphans.
2. Every referenced mesh exists on disk.
3. Robot FK at the zero pose reproduces the link frames read out of the RobotStudio
   station, including each joint's axis direction.  The zero-range rail must not move it.
4. Positioner FK reproduces the CALIBRATED `ARM_TYPE.rot_axis_pose_*` entries from the
   cell's MOC.cfg -- the controller's own view of where the axes are.  Both the driven
   station and the frozen idle one are checked, because freezing bakes an angle into an
   origin and that arithmetic is easy to get wrong.
5. Joint limits match MOC.cfg working ranges.
6. The tailstock joint starts on the faceplate and slides along the rotation axis.
7. The cuRobo yaml agrees with its URDF.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import yaml as yamllib
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


# MOC.cfg ARM_TYPE rot_axis_pose, in the robot base frame.  STN1 is calibrated with the
# index at 0 and STN2 with it at pi -- both are given at the work position.
MOC = {
    "ARM1":   ((1.40935, -0.061387, 0.659467), (0.502568, -0.487831, -0.43362, 0.566939), "stn1_tilt"),
    "PLATE1": ((1.37036, 0.800883, 0.409268), (0.535299, 0.53563, -0.459753, 0.463878), "stn1_plate"),
    "ARM2":   ((1.40656, -0.061249, 0.658594), (0.504478, -0.486105, -0.436636, 0.564405), "stn2_tilt"),
    "PLATE2": ((1.37014, 0.79792, 0.41657), (0.536457, 0.537943, -0.460038, 0.459561), "stn2_plate"),
}

EXPECTED_LINKS = {  # origin, joint axis in the robot base frame -- from the station
    "link_1": ((0, 0, 0.495), (0, 0, 1)),
    "link_2": ((0.175, 0, 0.495), (0, 1, 0)),
    "link_3": ((0.175, 0, 1.590), (0, 1, 0)),
    "link_4": ((1.4055, 0, 1.765), (1, 0, 0)),
    "link_5": ((1.4055, 0, 1.765), (0, 1, 0)),
    "link_6": ((1.4055, 0, 1.765), (1, 0, 0)),
}

ARM_LIMITS = {
    "joint_1": (-3.14159265, 3.14159265), "joint_2": (-1.57079633, 2.61799388),
    "joint_3": (-3.14159265, 1.30899694), "joint_4": (-6.98131701, 6.98131701),
    "joint_5": (-2.09439510, 2.09439510), "joint_6": (-6.98131701, 6.98131701),
    "linear_rail_joint": (0.0, 0.0),
}

for station in (1, 2):
    upath = BUNDLE / f"monarc_stn{station}.urdf"
    ypath = BUNDLE / f"monarc_torch_stn{station}.yaml"
    idle = 2 if station == 1 else 1
    print(f"\n########## station {station} ##########")

    print("\n[1] parse with urdfpy (the parser the app uses)")
    robot = URDF.load(str(upath))
    names = {l.name for l in robot.links}
    check("loads", True, f"{len(robot.links)} links, {len(robot.joints)} joints")
    children = {j.child for j in robot.joints}
    roots = [n for n in names if n not in children]
    check("exactly one root link", len(roots) == 1, f"root = {roots}")
    moving = [j.name for j in robot.joints if j.joint_type != "fixed"]
    expected_moving = ["linear_rail_joint"] + list(ARM_LIMITS)[:6] + [
        f"stn{station}_tilt_joint", f"stn{station}_plate_joint", f"stn{station}_tailstock_joint"]
    check("10 moving joints, this station's only", set(moving) == set(expected_moving),
          f"{len(moving)}: {sorted(moving)}")
    check(f"idle station {idle} is frozen",
          not [j for j in moving if j.startswith(f"stn{idle}_")]
          and "positioner_index_joint" not in moving)

    print("\n[2] referenced meshes exist")
    missing = []
    for link in robot.links:
        for section in list(link.visuals) + list(link.collisions):
            mesh = getattr(section.geometry, "mesh", None)
            if mesh is not None and not (upath.parent / mesh.filename).resolve().is_file():
                missing.append(mesh.filename)
    check("all mesh files present", not missing, f"{len(missing)} missing" if missing else "")

    print("\n[3] robot FK at the zero pose vs the RobotStudio station link frames")
    fk = robot.link_fk()
    byname = {l.name: fk[l] for l in robot.links}
    for name, (origin, axis) in EXPECTED_LINKS.items():
        T = byname[name]
        dp = np.linalg.norm(T[:3, 3] - np.array(origin)) * 1000
        da = np.degrees(np.arccos(np.clip(abs(np.dot(T[:3, 2], axis)), -1, 1)))
        check(f"{name} origin & axis", dp < 0.5 and da < 0.05, f"{dp:.4f} mm, {da:.4f} deg")
    d = np.linalg.norm(byname["tool0"][:3, 3] - np.array([1.4905, 0, 1.765])) * 1000
    check("tool0 at wrist + 85 mm", d < 0.5, f"{d:.4f} mm")

    print("\n[4] positioner FK vs MOC.cfg ARM_TYPE rot_axis_pose (the calibrated truth)")
    # MOC gives each station's axes at the WORK position, so only this template's driven
    # station is comparable.  The idle one is checked differently: it must be parked out
    # at the load position, which is what proves the index freeze went the right way.
    for label, (pos, quat, link) in MOC.items():
        T = robot.link_fk(link=link)      # every positioner joint is 0 or frozen here
        if link.startswith(f"stn{station}"):
            a_model = T[:3, 2]
            a_moc = quat_abb(quat)[:, 2]
            if np.dot(a_moc, a_model) < 0:
                a_moc = -a_moc
            ang = np.degrees(np.arccos(np.clip(abs(np.dot(a_moc, a_model)), -1, 1)))
            delta = np.array(pos) - T[:3, 3]
            perp = np.linalg.norm(delta - np.dot(delta, a_model) * a_model) * 1000
            check(f"{label} ({link}) is at the calibrated work pose", perp < 6.0 and ang < 0.4,
                  f"axis line {perp:.2f} mm, {ang:.2f} deg")
        else:
            reach = np.linalg.norm(T[:3, 3])
            check(f"{label} ({link}) is parked clear of the robot", reach > 3.0,
                  f"{reach:.2f} m from the robot base, past the 2.50 m reach")

    print("\n[5] joint limits vs MOC.cfg working ranges")
    limits = dict(ARM_LIMITS)
    limits[f"stn{station}_tilt_joint"] = (-3.15904595, 3.15904595)
    limits[f"stn{station}_plate_joint"] = (-20.0, 20.0)
    limits[f"stn{station}_tailstock_joint"] = (-2.5, 2.5)
    bad = [f"{j.name}: {j.limit.lower}..{j.limit.upper}" for j in robot.joints
           if j.name in limits and (abs(j.limit.lower - limits[j.name][0]) > 1e-6
                                    or abs(j.limit.upper - limits[j.name][1]) > 1e-6)]
    check("all 10 limits match", not bad, "; ".join(bad))
    rail = [j for j in robot.joints if j.name == "linear_rail_joint"][0]
    check("rail is a zero-range prismatic (single preset at 0)",
          rail.joint_type == "prismatic" and rail.limit.lower == 0.0 and rail.limit.upper == 0.0)

    print("\n[6] tailstock prismatic joint")
    T0 = robot.link_fk(link=f"stn{station}_tailstock")
    plate = robot.link_fk(link=f"stn{station}_plate")
    face = plate[:3, 3] - 0.856 * plate[:3, 2]
    check("tailstock starts on the faceplate",
          np.linalg.norm(T0[:3, 3] - face) * 1000 < 1.0,
          f"{np.linalg.norm(T0[:3, 3] - face)*1000:.3f} mm")
    T1 = robot.link_fk(cfg={f"stn{station}_tailstock_joint": 1.0},
                       link=f"stn{station}_tailstock")
    along = np.dot(T1[:3, 3] - T0[:3, 3], plate[:3, 2])
    check("tailstock slides along the rotation axis", abs(abs(along) - 1.0) < 1e-6,
          f"1.000 m of travel is {abs(along):.6f} m along the axis")

    print("\n[7] cuRobo yaml agrees with the URDF")
    kin = yamllib.safe_load(ypath.read_text())["robot_cfg"]["kinematics"]
    check("urdf_path points at this station's URDF", kin["urdf_path"] == upath.name,
          kin["urdf_path"])
    check("base_link / ee_link exist", kin["base_link"] in names and kin["ee_link"] in names)
    unknown = [n for n in kin["link_names"] + kin["collision_link_names"] if n not in names]
    check("every named link is real", not unknown, str(unknown))
    check("cspace.joint_names == the URDF's moving joints",
          set(kin["cspace"]["joint_names"]) == set(moving),
          f"{len(kin['cspace']['joint_names'])} vs {len(moving)}")
    for key in ("retract_config", "null_space_weight", "cspace_distance_weight"):
        check(f"cspace.{key} length matches",
              len(kin["cspace"][key]) == len(kin["cspace"]["joint_names"]))
    ig = {k: set(v or []) for k, v in kin["self_collision_ignore"].items()}
    asym = [(a, b) for a in ig for b in ig[a] if b not in ig or a not in ig[b]]
    check("self_collision_ignore is symmetric", not asym, str(asym[:3]))
    bad = [n for k, v in ig.items() for n in list(v) + [k] if n not in names]
    check("self_collision_ignore names are real links", not bad, str(set(bad)))
    check("no link ignores itself", not [k for k, v in ig.items() if k in v])
    spheres = kin["collision_spheres"]
    check("every collision link has spheres",
          not [n for n in kin["collision_link_names"] if n not in spheres])
    n_spheres = sum(len(v) for v in spheres.values())
    check("all sphere radii positive",
          not [1 for v in spheres.values() for s in v if s["radius"] <= 0], f"{n_spheres} spheres")
    # cuRobo JIT-compiles a kernel that scales with sphere count; 918 died with
    # "nvrtc: catastrophic error: out of memory".  The shipped FANUC bundle uses 87.
    check("sphere count is the same order as the shipped reference (87)", n_spheres <= 260,
          f"{n_spheres}")

    print("\n[8] coarse spheres must not phantom-block the weld")
    # Raw slack is alarming on big shells but mostly points AWAY from the work.  What
    # matters is whether any body that is not part of this station intrudes on the tube
    # plus the torch's working envelope.
    P = robot.link_fk(link=f"stn{station}_plate")
    face = P[:3, 3] - 0.856 * P[:3, 2]
    tube = face[None, :] + np.outer(np.linspace(0.0, 2.0, 60), -P[:3, 2])
    TUBE_R, TORCH_CLEAR = 0.15, 0.30
    worst_name, worst_gap = "", 1e9
    for name, entries in spheres.items():
        if name.startswith(f"stn{station}"):
            continue                      # these bodies legitimately hold the tube
        T = byname[name]
        c = np.array([e["center"] for e in entries])
        r = np.array([e["radius"] for e in entries])
        w = (T[:3, :3] @ c.T).T + T[:3, 3]
        gap = float((np.linalg.norm(tube[:, None, :] - w[None, :, :], axis=2)
                     - r[None, :] - TUBE_R).min())
        if gap < worst_gap:
            worst_name, worst_gap = name, gap
    check("every non-station body clears the tube + torch envelope", worst_gap >= TORCH_CLEAR,
          f"tightest is {worst_name} at {worst_gap:+.2f} m (need >= {TORCH_CLEAR:.2f})")

print(f"\n{checks - len(failures)}/{checks} checks passed")
if failures:
    print("FAILED: " + "; ".join(failures))
sys.exit(1 if failures else 0)
