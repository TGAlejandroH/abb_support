"""Emit the cuRobo robot config for the MONARC bundle.

Run with the Weld Planner's environment (urdfpy + open3d):
    ~/anaconda3/envs/pyoccenv/python.exe build_curobo_yaml.py

Reads  bundle/monarc_stn{1,2}.urdf + bundle/meshes/*.stl
Writes bundle/monarc_torch_stn1.yaml and bundle/monarc_torch_stn2.yaml

Collision spheres CONTAIN their link's surface (k-means on surface samples, each sphere
sized to hold its cluster).  Strictly-inscribed spheres were tried first and are wrong for
this cell: most parts are thin sheet-metal shells, a 3 mm cover panel admits no sphere at
all, and coverage came out at ~2%.  A gap in the model is a hole a torch can be planned
through; an oversized sphere only refuses poses that were actually fine.  So the model
over-approximates deliberately, and the script measures the cost as `slack`.

Over-approximating is exactly what broke the first ABB bundle, so `self_collision_ignore`
is built from MESH-level proximity rather than from sphere overlap -- ignoring a pair
because its spheres touch would launder the sphere model's own slack into the ignore map.
It is symmetric by construction.

Both stations share one sphere fit; the two configs differ only in `urdf_path` and
`cspace.joint_names`.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from urdfpy import URDF

HERE = Path(__file__).resolve().parent
BUNDLE = HERE / "bundle"

BASE_LINK = "base"
EE_LINK = "tcp_torch"


def urdf_path(station):
    return BUNDLE / f"monarc_stn{station}.urdf"


def yaml_path(station):
    return BUNDLE / f"monarc_torch_stn{station}.yaml"


def cspace_joints(station):
    """Rail first, then the arm, then the station -- the reference bundle's ordering."""
    return ["linear_rail_joint",
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
            f"stn{station}_tilt_joint", f"stn{station}_plate_joint",
            f"stn{station}_tailstock_joint"]

# Sphere count per link.  Slack falls roughly as 1/sqrt(count), so this trades collision
# fidelity against planner cost -- but the ceiling is hard: cuRobo JIT-compiles a kernel
# whose size scales with the sphere count, and an earlier 918-sphere model died with
# "nvrtc: catastrophic error: out of memory" before it could plan anything.  The shipped
# FANUC reference bundle runs on 87 spheres, so this budget aims at ~200: enough to model
# the positioner properly (the reference gives its positioner 0.002 m placeholders) while
# staying the same order as something known to compile.
#
# The positioner is part of the robot's own sphere model in cuRobo -- its joints are locked
# per solve, not removed -- so a loose bed pushes the torch away from the seam.  The
# weld-adjacent parts therefore get most of the budget.
COUNT = {
    "base_link": 8, "link_1": 8, "link_2": 8, "link_3": 8,
    "link_4": 10, "link_5": 6, "link_6": 4, "torch": 28,
    # The rotating wall is a 4.5 x 1.5 m panel only 42 mm thick, so every sphere on it
    # bulges toward the robot by very nearly its own radius.  At 10 spheres those radii
    # reached 881 mm and swallowed the workspace; the count has to be high enough that the
    # radius is small, and the wall prune below is what pays for it.
    "pos_cover": 40,
    # The far chunky bodies round the index axis: coarse is fine, they are 2.2-2.5 m out
    # and the wall prune already removed most of their surface.
    "positioner_base": 6, "pos_turntable": 5, "pos_bearing_stn1": 6, "pos_bearing_stn2": 4,
    "stn1_arm": 10, "stn1_bed": 20, "stn1_rotate_drive": 8, "stn1_plate": 14,
    "stn2_arm": 10, "stn2_bed": 20, "stn2_rotate_drive": 8, "stn2_plate": 14,
    "stn1_tailstock": 8, "stn2_tailstock": 8,
}
DEFAULT_COUNT = 8
SURFACE_SAMPLES = 8000
TOUCHING = 0.006          # meshes closer than this at the zero pose are in contact

# Reach-based pruning.  A link that is rigidly STATIC with respect to the robot base and
# lies wholly outside the arm's envelope can never be touched, so its spheres are pure
# kernel cost.  In each template that is most of the idle station -- exactly the operator
# side, which the robot has no access to.
#
# 3.360 m is measured, not assumed: the greatest distance from the robot base to any
# robot-side collision sphere, maximised over joints 2/3/5 (with 4 and 6 at 0 and 90 deg)
# across the full working ranges.  It already includes the torch and the sphere radii, so
# it is well past the 2.50 m nominal wrist reach.
REACH_MAX_M = 3.360
DROP_MARGIN_M = 0.10      # keep anything that comes within 100 mm of being reachable
FAR_BAND_M = 2.60         # static and this far out: clip it to what the arm can see

# The interchange cover is the cell's ROTATING WALL: a 42 mm-thin panel 4.54 m wide and
# 1.5 m tall, standing across the index axis and turning with the turntable.  It is what
# separates the robot from the operator, and geometry on its far side is the loading side
# the robot never works in.  Surface that the wall hides from the robot is therefore
# dropped: it is pure kernel cost, and coarse spheres out there bulge back toward the arm.
#
# This is a CELL constraint, not a pure reachability proof -- the wall is 1.5 m tall and the
# arm could in principle reach over it.  What stops that in the real cell is the SafeMove
# keep-in zone plus the stationary RAPID world zones (see joint_limits.md), and Monarch
# confirmed the robot has no access to the loading side.
WALL_LINK = "pos_cover"
WALL_EPS_M = 0.03         # the panel is 42 mm thick; keep its own skin


def link_meshes(robot, upath):
    """link name -> resolved mesh path, for links that carry collision geometry."""
    out = {}
    for link in robot.links:
        for c in link.collisions:
            mesh = getattr(c.geometry, "mesh", None)
            if mesh is None:
                continue
            out[link.name] = (upath.parent / mesh.filename).resolve()
    return out


def _scene(mesh):
    s = o3d.t.geometry.RaycastingScene()
    s.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return s


def wall_plane_for(meshes, world0):
    """(centre, normal) of the rotating wall in world coords; normal faces the robot.

    A plane, not a ray-occlusion test.  Ray casting from the robot base leaks under and
    around a finite panel -- it left the whole idle arm "visible" -- whereas the physical
    rule the cell actually obeys is simply which SIDE of the wall you are on.
    """
    if WALL_LINK not in meshes:
        return None, None
    m = o3d.io.read_triangle_mesh(str(meshes[WALL_LINK]))
    m.transform(world0[WALL_LINK])
    p = np.asarray(m.sample_points_uniformly(20000).points)
    c = p.mean(axis=0)
    _, V = np.linalg.eigh((p - c).T @ (p - c) / len(p))
    n = V[:, 0]                       # smallest spread = the panel normal
    if np.dot(-c, n) < 0:             # make it point at the robot base
        n = -n
    return c, n


def _robot_side_of_wall(world_pts, plane):
    """False where a point lies past the rotating wall, on the loading side."""
    c, n = plane
    if c is None or len(world_pts) == 0:
        return np.ones(len(world_pts), bool)
    return ((world_pts - c) @ n) >= -WALL_EPS_M


def containing_spheres(mesh_path, count, world_T=None, reach=None, wall=None):
    """Cluster the surface, then size each sphere to CONTAIN its cluster.

    Coverage is 100% by construction: every kept surface point lies inside some sphere.
    That is the property that matters -- a gap in the model is a hole a torch can be
    planned through, whereas an oversized sphere only refuses poses that were actually
    fine.  The cost of that safety is reported as `slack`: how far the union of spheres
    reaches beyond the real surface.

    `world_T` + `reach` clip the fit to the part of the surface that lies within `reach`
    of the robot base.  That matters for a big STATIC body sitting mostly out of the
    envelope: fitting the whole thing on a small budget makes metre-wide spheres that
    bulge back INTO the workspace and phantom-block the arm, while fitting only the
    reachable slice is both tighter and cheaper.  Returns no spheres when nothing is in
    reach, which is the caller's cue to drop the link.
    """
    from scipy.cluster.vq import kmeans2

    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    pts = np.asarray(mesh.sample_points_uniformly(SURFACE_SAMPLES).points)
    if world_T is not None:
        w = (world_T[:3, :3] @ pts.T).T + world_T[:3, 3]
        keep = np.ones(len(pts), bool)
        if reach is not None:
            keep &= np.linalg.norm(w, axis=1) <= reach
        if wall is not None:
            keep &= _robot_side_of_wall(w, wall)
        pts = pts[keep]
        if len(pts) == 0:
            return [], 0.0, 1.0
    k = int(min(count, max(1, len(pts) // 8)))

    centres, labels = kmeans2(pts, k, minit="++", seed=0, iter=40)
    spheres = []
    for i in range(len(centres)):
        member = pts[labels == i]
        if len(member) == 0:
            continue
        c = member.mean(axis=0)
        r = float(np.linalg.norm(member - c, axis=1).max())
        spheres.append((c, r))

    # containment check against the full sample, not just the cluster members
    cen = np.array([c for c, _ in spheres])
    rad = np.array([r for _, r in spheres])
    d = np.linalg.norm(pts[:, None, :] - cen[None, :, :], axis=2) - rad[None, :]
    covered = float((d.min(axis=1) <= 1e-9).mean())

    # slack = furthest any sphere reaches beyond the surface
    scene = _scene(mesh)
    slack = float(np.max(rad + scene.compute_distance(
        o3d.core.Tensor(cen.astype(np.float32))).numpy()))
    return spheres, slack, covered


def rigid_groups(robot):
    """map every collision link to the moving joint it is rigidly attached to."""
    parent_of, jtype = {}, {}
    for j in robot.joints:
        parent_of[j.child] = j.parent
        jtype[j.child] = j.joint_type
    group = {}
    for link in robot.links:
        n, chain = link.name, []
        while n in parent_of and jtype[n] == "fixed":
            chain.append(n)
            n = parent_of[n]
        group[link.name] = n
    return group, parent_of, jtype


def build(station, sphere_cache):
    upath, ypath = urdf_path(station), yaml_path(station)
    CSPACE = cspace_joints(station)
    print(f"\n########## station {station}: {upath.name} ##########")
    robot = URDF.load(str(upath))
    meshes = link_meshes(robot, upath)
    collision_links = [l.name for l in robot.links if l.name in meshes]

    # --- reach pruning ------------------------------------------------------------------
    group, parent_of, jtype = rigid_groups(robot)
    fk0 = robot.link_fk()
    world0 = {l.name: fk0[l] for l in robot.links}

    def static_nearest(name):
        """Distance from the robot base to the nearest point of a STATIC link, else None."""
        if group.get(name) != BASE_LINK:
            return None                      # a moving joint lies between; distance varies
        mesh = o3d.io.read_triangle_mesh(str(meshes[name]))
        pts = np.asarray(mesh.sample_points_uniformly(2000).points)
        T = world0[name]
        w = (T[:3, :3] @ pts.T).T + T[:3, 3]
        return float(np.linalg.norm(w, axis=1).min())

    print(f"[0] clipping STATIC links to the arm's envelope ({REACH_MAX_M:.2f} m + "
          f"{DROP_MARGIN_M*1000:.0f} mm)")
    reach = REACH_MAX_M + DROP_MARGIN_M
    wall = wall_plane_for(meshes, world0)
    budget, clip = {}, {}
    for name in collision_links:
        near = static_nearest(name)
        budget[name] = COUNT.get(name, DEFAULT_COUNT)
        # Only a link rigidly fixed to the base has a distance that holds in every pose.
        # The wall occludes nothing of itself, so it is clipped by reach alone.
        if near is not None and near > FAR_BAND_M:
            clip[name] = (world0[name], reach, None if name == WALL_LINK else wall)
            hidden = "" if name == WALL_LINK else " and drop what the wall hides"
            print(f"    {name:22} static, nearest {near:5.2f} m -> keep the reachable part{hidden}")
        else:
            clip[name] = (None, None, None)

    print(f"\n[1] fitting containing spheres to {len(collision_links)} collision links")
    spheres, worst_slack, worst_cover = {}, 0.0, 1.0
    dropped = []
    for name in list(collision_links):
        wt, rr, wl = clip[name]
        key = (name, budget[name], None if wt is None else station)
        if key not in sphere_cache:
            sphere_cache[key] = containing_spheres(meshes[name], budget[name], wt, rr, wl)
        sph, slack, cover = sphere_cache[key]
        if len(sph) == 0:
            dropped.append(name)
            collision_links.remove(name)
            print(f"    {name:22} DROPPED - no part of it is within the arm's envelope")
            continue
        worst_slack = max(worst_slack, slack)
        worst_cover = min(worst_cover, cover)
        spheres[name] = sph
        rad = [r for _, r in sph]
        print(f"    {name:22} {len(sph):3d} spheres  r {min(rad)*1000:3.0f}-{max(rad)*1000:3.0f} mm"
              f"   slack {slack*1000:5.1f} mm   contains {cover*100:6.2f}% of the surface")

    print(f"\n    worst slack {worst_slack*1000:.1f} mm   worst containment {worst_cover*100:.2f}%")
    if worst_cover < 0.9999:
        sys.exit("REFUSING TO EMIT: a link's spheres do not contain its surface")

    # --- self_collision_ignore, from MESH-level evidence, symmetric by construction ---
    print("\n[2] building a symmetric self_collision_ignore from mesh proximity")
    # (group, parent_of, jtype already resolved above for the reach prune)
    ignore = {n: set() for n in collision_links}

    def add(a, b):
        if a in ignore and b in ignore and a != b:
            ignore[a].add(b)
            ignore[b].add(a)

    # (i) parts rigidly bolted to the same moving link never move relative to each other
    for a in collision_links:
        for b in collision_links:
            if a < b and group[a] == group[b]:
                add(a, b)

    # (ii) links either side of one moving joint are in contact at the joint by design
    for a in collision_links:
        for b in collision_links:
            if a >= b:
                continue
            ga, gb = group[a], group[b]
            if parent_of.get(ga) == gb or parent_of.get(gb) == ga:
                add(a, b)

    # (iii) evidence: metal that is genuinely touching at the zero pose.  Measured on the
    # MESHES, not on the spheres -- ignoring a pair because its spheres overlap would
    # launder the sphere model's own over-approximation into the ignore map.
    fk = robot.link_fk()
    byname = {l.name: fk[l] for l in robot.links}
    world_scene, world_pts = {}, {}
    for n in collision_links:
        m = o3d.io.read_triangle_mesh(str(meshes[n]))
        m.transform(byname[n])
        world_scene[n] = _scene(m)
        world_pts[n] = np.asarray(m.sample_points_uniformly(1500).points, dtype=np.float32)

    evidence = 0
    for i, a in enumerate(collision_links):
        for b in collision_links[i + 1:]:
            if b in ignore[a]:
                continue
            d = world_scene[b].compute_distance(o3d.core.Tensor(world_pts[a])).numpy().min()
            if d < TOUCHING:
                add(a, b)
                evidence += 1
                print(f"    touching at the zero pose: {a} <-> {b}  ({d*1000:.1f} mm)")
    total = sum(len(v) for v in ignore.values()) // 2
    print(f"    {total} ignored pairs ({evidence} of them from mesh-level evidence)")
    asym = [(a, b) for a in ignore for b in ignore[a] if a not in ignore[b]]
    print(f"    symmetric: {not asym}")

    # --- emit -------------------------------------------------------------------------
    lim = {j.name: j.limit for j in robot.joints if j.joint_type != "fixed"}
    lines = [
        "# MONARC cell -- ABB IRB 4600-20/2.50 + IRBP D600 L2000 D1200.",
        "# Generated by build_curobo_yaml.py. Do not hand-edit.",
        "# Spheres CONTAIN their link's surface (100%% of sampled points), so the model has no",
        "# gaps; the price is up to %.0f mm of slack on the loosest link." % (worst_slack * 1000),
        "# self_collision_ignore is symmetric by construction and its evidence is mesh-level.",
        "robot_cfg:",
        "  kinematics:",
        "    use_usd_kinematics: false",
        "    usd_path: ''",
        "    usd_robot_root: ''",
        "    isaac_usd_path: ''",
        "    usd_flip_joints: {}",
        "    usd_flip_joint_limits: []",
        f"    urdf_path: {upath.name}",
        "    asset_root_path: .",
        f"    base_link: {BASE_LINK}",
        f"    ee_link: {EE_LINK}",
        "    link_names:",
    ]
    frame_links = [l.name for l in robot.links]
    lines += [f"    - {n}" for n in frame_links]
    lines += ["    lock_joints: null", "    extra_links: null", "    collision_link_names:"]
    lines += [f"    - {n}" for n in collision_links]
    lines += ["    collision_spheres:"]
    for n in collision_links:
        lines.append(f"      {n}:")
        for c, r in spheres[n]:
            lines.append(f"      - center: [{c[0]:.6f}, {c[1]:.6f}, {c[2]:.6f}]")
            lines.append(f"        radius: {r:.6f}")
    lines += ["    collision_sphere_buffer: 0.0", "    extra_collision_spheres: {}",
              "    self_collision_ignore:"]
    for n in collision_links:
        lines.append(f"      {n}:")
        for other in sorted(ignore[n]):
            lines.append(f"      - {other}")
    lines += ["    self_collision_buffer:"]
    lines += [f"      {n}: 0.0" for n in collision_links]
    lines += ["    use_global_cumul: true", "    mesh_link_names:"]
    lines += [f"    - {n}" for n in collision_links]
    lines += ["    cspace:", "      joint_names:"]
    lines += [f"      - {n}" for n in CSPACE]
    lines += ["      retract_config:"] + [f"      - {0.0}" for _ in CSPACE]
    lines += ["      null_space_weight:"] + ["      - 1.0" for _ in CSPACE]
    lines += ["      cspace_distance_weight:"] + ["      - 1.0" for _ in CSPACE]
    lines += ["      max_jerk: 500.0", "      max_acceleration: 15.0"]

    ypath.write_text("\n".join(lines) + "\n", encoding="utf-8")
    n_sph = sum(len(v) for v in spheres.values())
    print(f"\nwrote {ypath.relative_to(HERE)}   {n_sph} spheres over {len(collision_links)} links")
    missing = [j for j in CSPACE if j not in lim]
    if missing:
        sys.exit(f"cspace names not in the URDF: {missing}")


def main():
    # the sphere fit depends only on the meshes, which both stations share
    cache = {}
    for station in (1, 2):
        build(station, cache)


if __name__ == "__main__":
    main()

