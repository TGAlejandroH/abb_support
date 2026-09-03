# MONARC cell — building the `.tgs` URDF from the RobotStudio project (v1)

**Question asked:** can we construct the URDF for a MONARC `.tgs` template from what
RobotStudio gave us, falling back to IRB 4600 datasheet nominals for the robot?

**Answer:** yes — for both machines' internal kinematics *and* for their relative placement,
with no measurement campaign needed. We do **not** need the datasheet: the RobotStudio project
carries the same numbers in exact form and they cross-check several independent ways. The one
number that ties the two machines together — the robot-base → positioner-base transform —
turned out to be **calibrated in the cell's own `MOC.cfg`**, under `ARM_TYPE.rot_axis_pose_*`
rather than the `base_frame` fields everyone looks for (§6c). Fitting to it agrees with the
RobotStudio station to **≤ 4.2 mm and ≤ 0.27°**.

**Status: built and verified.** Two `.tgs` templates, a URDF and a cuRobo config per station,
23 shared meshes — see §10 for what exists and how each number was checked.

Read with [joint_limits.md](joint_limits.md) (working ranges, already surveyed) and
`curobo_suite/docs/abb_integration_plan_v1.md` (**D4** already locks one `.tgs` per station).

---

## 1. Sources read

| Source | What it yielded |
|---|---|
| `D:\ABB\Monarc_RS\Project\Station\Monarch_1.2R2.rsstnx` | **Plain XML** (11 MB, `PIMDocument`). Robot mechanism definition incl. full DH, joint limits, per-link correction transforms; station placement of robot + positioner + torch; `MechanicalUnit` base frames |
| `…\Project\Components\IRBP_D600_D1200-L2000_M2009_REV1_01.rslib` | **ZIP** containing `PIM.xml` (43 KB). Positioner DH for all 5 joints, kinematic base frame, per-link correction transforms, per-part placements, the two workpiece flange frames |
| `…\Controller Data\4600-803651_Virtual\SYSPAR\MOC.cfg` | Mech-unit ↔ joint ↔ logical-axis mapping; working ranges; and — under `ARM_TYPE`, not `base_frame` — **the calibrated pose of every station axis in the robot's frame** (§6c) |
| `…\RAPID\TASK1\SYSMOD\wobj_Database.sys` | Four taught station work objects; proves the cell already welds coordinated and corroborates the mast tilt to 0.02° (§6b) |
| `D:\ABB\CAD Models\**\OBJ\*.obj` | Link meshes, metres, right-handed |
| `D:\ABB\CAD Models\TGS Samples Projects\*.tgs` | The Weld Planner's URDF + bundle contract |

Both `.rsstnx` and `.rslib/PIM.xml` are readable with nothing but Python — no RobotStudio,
no API. That matters: this is a repeatable extraction, not a one-off manual trace.

---

## 2. What a `.tgs` is, concretely

A `.tgs` is a **SQLite database** (`SQLite format 3` magic, `project_meta.format = tgs-sqlite`,
`schema_version = 8`). The robot model lives in it as ordinary rows:

- **`robot_bundle_files(path, content_blob, sha256)`** — the whole bundle embedded: the
  `.urdf`, the cuRobo `.yaml`, every mesh, plus generated artefacts (`HMI_CACHE/`,
  `ROBOT_PROGRAM/`, `WELDS/`).
- **`project_meta`** — `robot_bundle_mode = embedded`, `robot_urdf_bundle_path`,
  `robot_yaml_bundle_path`, `robot_bundle_root`.
- **`robot_groups`** — `arm_joint_json`, `base_link`, `tcp_link`, `robot_brand`,
  `output_profile_id`, `home_poses_json`.
- **`external_axis_groups`** — `joint_json`, `kind` (`rail` / `positioner`), `shared`,
  `ownership_mode`, `positioner_base_link`, `positioner_ee_link`, and the
  `metadata_json.export_v1.brands.<brand>` axis-export mapping.
- **`nodes`** — scene tree; a frame can be parented straight to a URDF link with
  `parent_ref_type='urdf_link'`, `parent_ref_value='<link name>'`.

⚠ **A `.tgs` embeds its own copy of the bundle.** Fixing the bundle on disk does nothing for
an existing project — it must be re-imported or the row patched. (Already cost the Weld
Planner track a full debugging cycle; recorded in their tracker.)

---

## 3. The Weld Planner URDF contract

Read off `two_axis_sample.tgs` (`m710ic12l.urdf` + `fanuc_torch_unlocked.yaml`), which is the
known-good 6-axis + rail + 2-axis-positioner case — structurally what MONARC needs, minus
the rail.

**One URDF holds the robot *and* the positioner.** They share a single root link `base`; the
positioner hangs off it through a `fixed` joint, and *that fixed joint is where the
robot↔positioner transform lives*:

```xml
<joint name="base-to-positioner_pedestal" type="fixed">   <!-- pedestal at the robot base -->
<joint name="positioner_tilt_joint" type="revolute">
  <origin rpy="0 -1.5707963267948966 0" xyz="1.75 0 0"/>  <!-- THE transform -->
```

Conventions the app relies on:

| Item | Convention |
|---|---|
| Units | metres, radians |
| Root link | `base` (`robot_groups.base_link`) |
| Arm chain | `base_link`, `link_1` … `link_6`; `robot_groups.tcp_link = link_6` (app convention, not a misconfiguration) |
| ROS-I frames | `flange`, then `tool0` via a fixed joint |
| Tooling | `torch`, `nozzle_id_0/1`, `tcp_torch` — all fixed to `tool0`; mirrored as rows in `tools` |
| Positioner | `positioner_pedestal` (static) → `positioner` (axis 1) → `positioner_frame` (axis 2); `positioner_ee_link = positioner_frame` is where the workpiece attaches |
| cuRobo YAML | `base_link`, `ee_link`, `link_names`, `collision_link_names`, `collision_spheres`, `self_collision_ignore`, `mesh_link_names`, `cspace.joint_names` — **`cspace.joint_names` order defines the joint vector** (rail, arm 1-6, positioner 1-2) |

Two traps, both already paid for once:

- **`self_collision_ignore` must be symmetric and evidence-based.** The first ABB bundle
  self-collided in every pose because it was one-directional and adjacent-only;
  `link_4`↔`link_6` overlapped 86 mm at the home pose.
- **Meshes are effectively STL-only.** The URDF editor's file dialog offers `.obj/.dae/.ply`
  and `import_mesh_into_bundle()` just copies the file, but the viewer renders URDF visuals
  through `occ_stl_visual.read_stl_as_mesh_shape()` → OCC's `rwstl.ReadFile` — **STL only**.
  Ship STL.

### The mesh unit trap — this one will bite

`CustomSimulatorDock._infer_stl_length_scale()` decides mm-vs-metres like this:

1. path contains `fanuc_stl_m/` → metres; contains `fanuc_stl/` → mm;
2. filename ends **`_m.stl`** → metres;
3. otherwise: read the binary STL header, and if `max |coord| < 2.0` assume metres.

MONARC breaks rule 3. If we follow the FANUC sample's own convention of authoring meshes in
the *model* frame (the sample does exactly this — `<origin xyz="0 0 -0.565">` subtracts each
link's zero-pose origin), coordinates reach **2.64 m** on the positioner bed and **1.77 m**
on the robot. Anything over 2.0 m is silently classified as millimetres and rendered 1000×
too small.

→ **Name every MONARC mesh `*_m.stl`.** It is the only brand-neutral hint that exists.

---

## 4. IRB 4600-20/2.50 — extracted and validated

`inverse(MechanismLinkInstance.CorrectionTransform)` in the station **is** each link's
kinematic frame in the robot base frame at zero pose. Each frame's **local Z is its joint
axis**:

| Link | Origin in robot base frame (m) | Joint axis (in base frame) | ABB axis |
|---|---|---|---|
| `Base` | 0, 0, 0 | — | — |
| `Link1` | 0, 0, 0.495 | (0, 0, 1) | 1 |
| `Link2` | 0.175, 0, 0.495 | (0, 1, 0) | 2 |
| `Link3` | 0.175, 0, 1.590 | (0, 1, 0) | 3 |
| `Link4` | 1.4055, 0, 1.765 | (1, 0, 0) | 4 |
| `Link5` | 1.4055, 0, 1.765 | (0, 1, 0) | 5 |
| `Link6` | 1.4055, 0, 1.765 | (1, 0, 0) | 6 |

As URDF joint origins (child frame in the parent link frame, every `axis` = `0 0 1`):

```
base   -> link_1   xyz="0 0 0.495"        rpy="0 0 0"
link_1 -> link_2   xyz="0.175 0 0"        rpy="-1.5707963 -1.5707963 0"
link_2 -> link_3   xyz="1.095 0 0"        rpy="0 0 0"
link_3 -> link_4   xyz="0.175 1.2305 0"   rpy="-1.5707963 0 0"
link_4 -> link_5   xyz="0 0 0"            rpy="-1.5707963 0 3.1415927"
link_5 -> link_6   xyz="0 0 0"            rpy="1.5707963 0 0"
link_6 -> tool0    xyz="0 0 0.085"        rpy="0 0 0"          (fixed)
```

**Validation — three independent agreements:**

1. The station's `ForwardKinematicsDH` block gives twist/length/rotation/offset
   `(0,0,0,0) (-90°,0.175,-90°,0) (0,1.095,0,0) (-90°,0.175,0,1.2305) (90°,0,-180°,0) (90°,0,0,0)`
   with `KinematicBaseFrames = (0,0,0.495)`. Composing that chain reproduces the table exactly.
2. Those are the IRB 4600 nominals: 495 mm base→axis 2 height, 175 mm offset, 1095 mm
   axis 2→3, 1230.5 + 175 mm axis 3→wrist. **The datasheet assumption is correct — and now
   redundant.**
3. `tool0` at wrist + **85 mm** is confirmed twice over: the `TWeldgun` mechanism is parked in
   the station at exactly `(1.4905, 0, 1.765)` in robot base coordinates, and the very first
   vertex of `Link6.obj` is `x = 1.4905`.

Joint limits from `ForwardKinematicsDH.JointLimitsVector` — ±180°, +150/−90°, +75/−180°,
±400°, ±120°, ±400° — match `MOC.cfg` exactly, so [joint_limits.md](joint_limits.md) stands.

---

## 5. IRBP D600 L2000 D1200 — extracted and validated

Five joints in one mechanism, shared across three mechanical units (`MOC.cfg`):

| Mech unit | Mechanism joints | Logical axes |
|---|---|---|
| `INTERCH` | 0, 2, 4 | 10, 8, 9 |
| `STN1` | 1, 2 | 8, 9 |
| `STN2` | 3, 4 | 8, 9 |

Link frames in the positioner model frame, same derivation as the robot:

| Link | Parts | Origin (m) | Joint axis | Role |
|---|---|---|---|---|
| `Base` | `Intch2000` | — | — | interchange housing |
| `Link1` | `Turntable500D`, `MTD2000` ×2, `Cover600D_D1200_L2000` | 1.310, −0.568, 1.250 | (0, 0, 1) | index / turntable |
| `Link2` | `Arm500D_2000`, `Bed`, `MTD750` | 0, −0.856, 1.250 | (−1, 0, 0) | **STN1 tilt** (`ARM1`) |
| `Link3` | `Headstock` | 0, −0.856, 1.000 | (0, −1, 0) | **STN1 rotate** (`PLATE1`) |
| `Link4` | `Arm500D_2000`, `Bed`, `MTD750` | 2.619, −0.280, 1.250 | (1, 0, 0) | **STN2 tilt** (`ARM2`) |
| `Link5` | `Headstock` | 2.619, −0.280, 1.000 | (0, 1, 0) | **STN2 rotate** (`PLATE2`) |

As URDF joints (every `axis` = `0 0 1`):

```
base   -> Link1   xyz="1.31 -0.568 1.25"     rpy="0 0 -1.5707963"          index
Link1  -> Link2   xyz="0.288 -1.31 0"        rpy="1.5707963 1.5707963 0"   STN1 tilt
Link2  -> Link3   xyz="0.25 0 0"             rpy="-1.5707963 0 0"          STN1 rotate
Link1  -> Link4   xyz="-0.288 1.309 0"       rpy="-1.5707963 1.5707963 0"  STN2 tilt
Link4  -> Link5   xyz="0.25 0 0"             rpy="-1.5707963 0 0"          STN2 rotate
```

Workpiece flange (`AttachmentPoints` frames `Irbp600D_1` / `Irbp600D_2`), identical on both
stations: `xyz="0 0 -0.856" rpy="0 0 3.1415927"` in the plate link frame.

**Validation — the geometry closes on itself:**

- The library's DH offsets (`d = 1.31 / 1.309`, `a = 0.288`, `a = 0.25`) reproduce those joint
  origins term for term.
- The two stations are exact mirrors about the index axis: tilt axes at y = −0.856 and
  y = −0.280, index axis at y = −0.568 — the precise midpoint. The 1 mm asymmetry
  (1.310 vs 1.309) is in ABB's own library, not our arithmetic.
- Joint 0's DH has two branches, the second carrying `Rotation = −π`: the 180° interchange.
- `MTD2000` sits on the STN1 tilt axis line and `MTD2000_2` on the STN2 one.

Working ranges: index **[−1°, +181°]**, tilt **±181°**, plate ±3600° in the library but
**±1145.92°** in this cell's `MOC.cfg` — the controller value wins.

---

## 6. Robot base → positioner base

### 6a. The 8.45° mast tilt is real, and the cell data proves it

`MechanismInstance` transforms, resolved into the robot base frame:

- robot base in station: `Rz(−90°)`, `t = (−0.338653, 3.66239, 1.23636)`
- positioner in station: rotation carrying an **8.449° tilt**, `t = (0.446908, 2.44184, 0.653225)`

The index axis (mast) therefore leans **8.459° off vertical**, toward azimuth 3.3° in the robot
base frame. That is deliberate, not modelling error: the interchange turntable is built on an
incline so the station facing the operator sits low for loading while the station facing the
robot sits high for welding. Confirmed visually in `D:\ABB\RobotPositionerImage.png` — the mast
is plainly canted and the two arms hang at different heights.

⚠ *An earlier revision of this doc called the tilt impossible on the grounds that the two
stations end up 389 mm apart in height. That 389 mm is the whole point of the design.*

The consequence, which the URDF must reproduce:

| Frame | Position in robot base frame (m) | Height |
|---|---|---|
| index axis | 2.705, 0.239, 0.465 | |
| STN1 plate (weld side, index 0) | 1.376, −0.062, 0.412 | **0.412 m** |
| STN2 plate (load side, index 0) | 3.961, 0.536, 0.023 | **0.023 m** |

Indexing 180° swaps them, so the station at the robot is always the high one.

### 6b. A work object taught on the real cell corroborates it

`RAPID/TASK1/SYSMOD/wobj_Database.sys` in the controller backup carries four taught work
objects — a coordinated and a non-coordinated twin per station:

```
wobj_Stn1        := [FALSE, FALSE, "STN1", [[1278.3,  640.454, 421.695], [0.00111738,-0.0736092,-0.00164098,-0.997285]], [[0.160774, 92.3662, 158.884], …]]
wobj_Stn2        := [FALSE, FALSE, "STN2", [[1279.86, 641.101, 425.564], [0.00105682,-0.0746829, 0.00211681,-0.997205]], [[-2.17704, 93.1062, 156.445], …]]
wobj_Stn1_NoCoord := same numbers, ufprog TRUE
wobj_Stn2_NoCoord := same numbers, ufprog TRUE
```

Three things fall out, and the second is the important one:

1. **`ufprog:=FALSE`, `ufmec:="STN1"/"STN2"` — this cell already welds coordinated**, exactly
   the model **D3** assumes. Independent confirmation, from the customer's own RAPID.
2. **The taught `uframe`'s Z axis is 8.44° off the robot base Z. The station model's mast is
   8.459°.** Those agree to **0.02°**, from two completely independent sources — a CAD station
   file and a frame taught by touching the real machine. The tilt is settled.
3. **`wobj_Stn1` and `wobj_Stn2` differ by only 4.2 mm.** Both stations present the same frame
   to the robot once indexed in — which is what an interchange positioner is supposed to do,
   and a good internal consistency check on the taught values.

These are *not* a kinematic snapshot of an identity base frame: reconstructing the STN1 chain
from the positioner's own kinematic base misses the taught origin by 1.96 m over every tilt
angle. They encode real cell geometry.

### 6c. SOLVED — the calibration is in `MOC.cfg`, under `ARM_TYPE`, not `base_frame`

⚠ *An earlier revision of this doc said the controller carries no positioner calibration,
because `MOC.cfg`'s `ROBOT` records have no `-base_frame_pos_*` / `-base_frame_orient_*` in
any of the three backups, every `MechanicalUnit/BaseFrame` is identity, and `STN1`/`STN2`/
`INTERCH` all carry `SkipBaseFrameCheck`. All of that is true, and all of it is a red
herring.*

**The calibration lives in `MOC.cfg`'s `ARM_TYPE` section.** Each station axis carries
`rot_axis_pose_pos_x/y/z` and `rot_axis_pose_orient_u0..u3` — its axis of rotation expressed
**directly in the robot's frame**:

| Arm type | `rot_axis_pose_pos` (m) | quaternion |
|---|---|---|
| `ARM1` | 1.40935, −0.061387, 0.659467 | 0.502568, −0.487831, −0.43362, 0.566939 |
| `PLATE1` | 1.37036, 0.800883, 0.409268 | 0.535299, 0.53563, −0.459753, 0.463878 |
| `ARM2` | 1.40656, −0.061249, 0.658594 | 0.504478, −0.486105, −0.436636, 0.564405 |
| `PLATE2` | 1.37014, 0.79792, 0.41657 | 0.536457, 0.537943, −0.460038, 0.459561 |

That is *why* there is no base frame and why `SkipBaseFrameCheck` is set: for a positioner
parameterised this way ABB does not use one. `ARM2`/`PLATE2` are given at the **work
position** (index = π), which is why they sit on top of `ARM1`/`PLATE1` — the same reason
`wobj_Stn1` and `wobj_Stn2` agree to 4.2 mm.

Against the station model, before any fitting:

| | axis line offset | axis direction |
|---|---|---|
| `ARM1` | 1.62 mm | 0.42° |
| `PLATE1` | 4.49 mm | 0.35° |
| `ARM2` | 2.16 mm | 0.35° |
| `PLATE2` | 10.90 mm | 0.73° |

So the RobotStudio station was accurate all along, and the earlier "0.1–0.2 m" estimate was
pessimistic — it came from comparing a faceplate frame at unknown joint angles.

Least-squares fitting the positioner placement to those four calibrated axes moves it by only
**(2.1, 9.4, 6.6) mm and 0.616°**, and leaves residuals of **0.25 / 3.39 / 1.04 / 4.16 mm** and
**≤ 0.27°**. The remaining few millimetres are real as-built asymmetry — the same asymmetry
that shows up as 1.310 vs 1.309 in ABB's own library.

**The fitted transform is what the URDF ships:**

```
base -> positioner_base   xyz="1.222672 0.794993 -0.576539"
                          rpy="0.000587 0.147569 0.002287"     (0.034, 8.455, 0.131 deg)
```

No pendant work, no layout drawing, no re-calibration. **Q1 and Q2 are closed.**

⚠ Only the four *station* axes are calibrated this way. `INTERCH`, `INTERCH_PLATE1` and
`INTERCH_PLATE2` have no `ARM_TYPE` entry in any backup — they live in the encrypted
`SEC_D600_L2000_D1200_TYPEA_STN1.cfg.enc` loaded `-internal`, the same file that hides the
`INTERCH` bounds ([joint_limits.md](joint_limits.md)). That costs us nothing, because the
index axis is held fixed per template.

---

## 7. Mesh pipeline

**Robot OBJs are in the robot model frame, Y-up:** `OBJ (x, y, z) = model (x, z, −y)`. Proven by
`Link6.obj` sitting at `(1.4905, 1.765, ~0)` against the model-frame flange at
`(1.4905, 0, 1.765)`, and by the progressive stacking of `LINK1`…`LINK6` along OBJ `y`. All
meshes have **positive signed volume**, so they are right-handed and not mirrored — but the
sign of the third axis should still be eyeballed on first import, because a mirrored robot is
instantly obvious and cheap to catch.

**Positioner OBJs are in each part's own local frame** — `LINK2/Bed.obj` and `LINK4/Bed.obj`
are byte-identical. They therefore need the placement transforms from `PIM.xml`. Those,
resolved into each link's frame, are:

```
Base   Intch2000              xyz="1.31 -0.568 0.481"    rpy="0 0 1.5707963"
Link1  Turntable500D          xyz="-0.476 -1.145 -1.25"  rpy="0 0 1.5707963"
Link1  MTD2000                xyz="0.288 -0.363 0"       rpy="1.5707963 -1.5707963 0"
Link1  MTD2000_2              xyz="-0.288 0.363 0"       rpy="-1.5707963 -1.5707963 0"
Link1  Cover600D_D1200_L2000  xyz="-0.568 -1.31 -1.25"   rpy="0 0 1.5707963"
Link2  Arm500D_2000           xyz="0 0 -0.947"           rpy="0 0 3.1415927"
Link2  Bed                    xyz="0.09 -1.138 -0.465"   rpy="-3.1415927 1.5707963 0"
Link2  MTD750                 xyz="0.25 -0.975 0"        rpy="-1.5707963 -1.5707963 0"
Link3  Headstock              xyz="0 0 -0.856"           rpy="0 0 -1.5707963"
Link4  Arm500D_2000           xyz="0 0 -0.946"           rpy="0 0 3.1415927"
Link4  Bed                    xyz="0.09 -1.138 -0.464"   rpy="-3.1415927 1.5707963 0"
Link4  MTD750                 xyz="0.25 -0.974 0"        rpy="-1.5707963 -1.5707963 0"
Link5  Headstock              xyz="0 0 -0.856"           rpy="0 0 -1.5707963"
```

### One mesh and one colour per link — a hard viewer constraint

`CustomSimulatorDock` resolves visuals through `_resolve_visual_mesh_for_link(link.name)` and
`_resolve_visual_color_for_link(link.name)`, both keyed on **link name**, both taking the
**first** entry. A link with several `<visual>` elements therefore renders its first mesh
N times, in one colour — the rest are invisible.

So a link cannot show multi-part CAD in its true colours. The way to keep colour is to give
each CAD part **its own URDF link**, joined to its kinematic parent by a `fixed` joint. That
satisfies the one-mesh-per-link rule, preserves each part's `Kd`, and gives the collision
backend finer bodies (it already does component-wise compound-convex on positioner links).
Cost: more links in the cuRobo YAML's `link_names` / `collision_link_names` /
`self_collision_ignore`, and more sphere work — which **M3** already requires anyway.

The robot needs none of this: RobotStudio exported one part per robot link already.

### Triangle budget — the positioner CAD must be decimated

| | MONARC as exported | FANUC reference bundle |
|---|---|---|
| robot links (6) | 7.5 k – 24.8 k each | 5 k – 34 k each |
| torch | 81.6 k | 350.7 k |
| positioner `Bed` ×2 | **753,734 each** | — |
| positioner `Headstock` ×2 | **328,500 each** | — |
| positioner total | **2.16 M** | **780** (3 box primitives) |
| grand total | **2.42 M** | 573 k |

The robot links are fine as they stand. `Bed` and `Headstock` are 89 % of the payload and must
be decimated hard — the working reference gets by with 168/324/288-triangle boxes for its whole
positioner, and the collision backend converts `positioner_pedestal` / `positioner_frame` to
compound-convex geometry anyway.

⚠ The station also carries `IRB4600_20kg-250_CABLES_LINK1/2/3_rev03` parts that were **not**
exported to OBJ. Dress packs are a real collision hazard around a torch — worth exporting.

### Done: `build_meshes.py`

[`build_meshes.py`](build_meshes.py) implements all of the above and has been run.
21 meshes into `bundle/meshes/`, **2,415,342 → 320,739 triangles, 16.0 MB** — the same order
as the FANUC reference bundle's 573 k. `Bed` 753,734 → 20,000 and `Headstock` 328,500 → 15,000
by quadric decimation (open3d); everything else is untouched.

Each mesh is metres, binary STL, named `*_m.stl`, and authored **in its own link's frame** so
the URDF visual/collision origin is identity. `bundle/meshes_manifest.json` carries the link
assignment, the source OBJ, the `Kd` colour, triangle counts and bounding box for every part.

**Verified**: pushing each converted STL back out through its link frame reproduces the
original CAD bounding box to **0.000 mm** for 19 of 21 parts. The two exceptions are the
decimated headstocks at 0.590 mm, which is quadric decimation shaving a bbox corner.

Colours recovered from the `.mtl` files — mostly ABB's greys, with `link_3` red
(0.996, 0, 0), both headstocks olive (0.753, 0.753, 0) and the interchange base dark slate
(0.282, 0.329, 0.349).

---

## 8. Workholding: the tailstock moves, so it cannot be a URDF link

**How MONARC actually works:** circular tubes of varying length are mounted **between the
headstock and the tailstock**. The operator slides the tailstock along the bed by hand to suit
the tube. No cantilevered setups. So the bed and both ends are always in play — the §7
decimation of `Bed` is not optional — and *the tailstock's position along the bed is a
per-part quantity that the Weld Planner has to carry somehow.*

A URDF link cannot express it: the tailstock would be frozen at one offset for every project
built from the template.

Three ways out, and the trade is about which link the tailstock rides:

| Option | Rides | Verdict |
|---|---|---|
| **Fixture box** (the suggestion) | the **plate** (rotate) axis, because fixtures bind to a scene frame under the workpiece chain | Placeable per project, no URDF change — but the tailstock then **spins with the tube**. It is a housing bolted to the bed, not axisymmetric, so it would sweep a large false collision volume. |
| **Prismatic joint on the tilt link**, locked per project | the **tilt** link — kinematically correct | Correct geometry and a single number per project. Cost: a new joint the exporter must never command. cuRobo locks external joints by name so planning is fine, but it needs an explicit `not_exported` mapping so it never reaches `extjoint`. |
| **Fixed link at a nominal offset**, edited per part family via the structured URDF editor | the tilt link — correct | Simplest, but re-editing the bundle per tube length is exactly the friction the fixture idea was avoiding. |

**Recommendation: the prismatic joint.** It is the only option that puts the tailstock on the
right link *and* keeps it a per-project number rather than a per-project bundle edit. The
`not_exported` export mode already exists for precisely this (**D23** keeps it un-rewritten),
and the collision backend already converts positioner links to compound-convex geometry.

If that is too much for a first template, ship the **fixture box** and accept the spin — a box
sized to the tailstock and centred on the rotation axis is conservative and harmless, it just
over-reserves space. Do **not** bake a fixed tailstock into the URDF.

⚠ Either way the *live centre* — the part of the tailstock that does rotate with the tube —
should be modelled with the plate link, and the housing with the tilt link.

---

## 9. Two templates, one bundle

**D4** already locks a separate `.tgs` per station. Both should embed the **same** URDF — the
full 5-joint positioner is one machine and the mesh payload is identical — and differ only in
`external_axis_groups`:

| | Station 1 template | Station 2 template |
|---|---|---|
| `joint_json` | `["stn1_tilt_joint","stn1_plate_joint","stn1_tailstock_joint"]` | the `stn2_*` equivalents |
| `positioner_ee_link` | `stn1_plate` | `stn2_plate` |
| index joint | parked at 0 rad | parked at π rad |
| `in` → `CAD` frame | on `stn1_plate` | on `stn2_plate` |

The workpiece chain is `urdf_link(stnN_plate)` → `in` → `CAD`, matching the two_axis sample.
`in` carries the −0.856 m faceplate offset; `CAD` is identity under it. Because the whole
chain hangs off the plate link, anything loaded under `CAD` rides the tilt **and** the
rotate/chuck axis — verified by driving each joint 0.30 rad and watching the frame move
(267 mm / 17.2° on tilt; 17.2° of pure rotation on the chuck, since the faceplate centre sits
exactly on that axis).

Treating the turntable as fixed (the stated simplification) is right, and also matches **D24**:
we never command the index axis, and cuRobo locks every external positioner joint by name
during arm solves, so an extra static joint costs nothing.

⚠ One thing this exposes: fixing the index at 0 for one template and π for the other means
**the two templates see different robot↔positioner geometry**, both derived from the single
transform of §6. Getting that transform wrong is wrong twice, in different directions — and
because the mast is tilted, the two errors are not even the same shape.

---

## 10. The build — what exists and how it was verified

Four scripts, run in order. All numbers below are measured, not estimated.

| Script | Output | Environment |
|---|---|---|
| [`build_meshes.py`](build_meshes.py) | `bundle/meshes/*_m.stl` + `meshes_manifest.json` | any env with open3d |
| [`build_urdf.py`](build_urdf.py) | `bundle/monarc_stn{1,2}.urdf` | stdlib only |
| [`build_curobo_yaml.py`](build_curobo_yaml.py) | `bundle/monarc_torch_stn{1,2}.yaml` | `pyoccenv` (urdfpy + open3d) |
| [`build_tgs.py`](build_tgs.py) | `bundle/MONARC_Station{1,2}.tgs` | `pyoccenv` |
| [`verify_bundle.py`](verify_bundle.py) | 68/68 checks | `pyoccenv` |
| [`verify_tgs.py`](verify_tgs.py) | 67/67 checks | `pyoccenv` |

**URDF** — one per station, 31 links, 30 joints, **10 moving** (rail, six arm, tilt, rotate, tailstock) against the reference project's 9. Robot FK at the zero pose reproduces the station's
link frames to **0.0000 mm**; `tool0` lands on wrist + 85 mm exactly. Positioner FK matches the
controller's calibrated `rot_axis_pose` entries to **0.25 / 3.39 / 1.04 / 4.16 mm** and ≤ 0.27°.
All 13 joint limits match `MOC.cfg`.

**Collision spheres** — **201 spheres over 18 links**, against the shipped FANUC reference
bundle's 87. An earlier 918-sphere model killed cuRobo outright with
`nvrtc: catastrophic error: out of memory`: the kernel it JIT-compiles scales with sphere
count, so the count is a hard ceiling, not a preference.

Three rules keep the model honest at that budget:

* **The rotating wall prunes the loading side.** `pos_cover` is not really a cover — it is a
  42 mm-thin panel **4.54 m wide and 1.5 m tall** standing across the index axis and turning
  with the turntable, and it is what separates the robot from the operator. Fitting its plane
  and dropping static surface on the far side removes **all five idle-station bodies**
  outright (they are 100% past it) and 29–82% of the turntable, base and bearings. A ray-cast
  occlusion test was tried first and is wrong: rays from the robot base leak under and around
  a finite panel, and it left the whole idle arm "visible". ⚠ This is a *cell* constraint, not
  a reachability proof — the wall is only 1.5 m tall. What stops the arm going over it is the
  SafeMove keep-in zone and the RAPID world zones ([joint_limits.md](joint_limits.md)).
* **Envelope clipping.** The greatest distance from the robot base to any robot-side sphere,
  maximised over the working ranges with torch and sphere radii included, is **3.360 m**
  (measured, not assumed). Static links are fitted only to the surface inside it.
* **A clearance test instead of a slack budget.** Raw slack looks alarming on big thin shells,
  but most of it points away from the work. What matters is whether a body that is not part of
  this station intrudes on the tube plus the torch's working envelope. Against a 300 mm tube
  over 2 m of rotation axis plus 300 mm of torch clearance, **every non-station body clears by
  ≥ 0.79 m**. That is a permanent check.

The wall was the worst offender before this pass: at 10 spheres its radii reached **881 mm** on
a 42 mm panel, swallowing the workspace. Pruning the loading side paid for 40 spheres on what
remains, bringing it to **223–324 mm** and more than doubling the tube clearance (+0.34 → +0.79 m).

Spheres still **contain** their (clipped) surface, 100.00% on every link, so the model has no
gaps. `self_collision_ignore` is symmetric by construction and its evidence is **mesh-level**
proximity, not sphere overlap — including `link_4` ↔ `link_6`, whose metal really is 3.1 mm
apart. That is the pair whose 86 mm sphere overlap broke the first ABB bundle.

**`.tgs`** — two templates, 16.5 MB each, 25 embedded bundle files. Verified that **no joint is
unassigned**, that the embedded copy is byte-identical to the verified on-disk bundle (a `.tgs`
carries its own copy, so this is worth asserting), that both templates embed the same **meshes**
and differ only in URDF + cuRobo config, and that FK run on the URDF *as extracted from the
`.tgs`* still lands on the calibrated faceplate. The app's own headless
reader (`curobo_suite/tools/tgs_fixture_report.py`) opens both.

---

## 11. Decisions taken (2026-09-01)

| | Decision | Why |
|---|---|---|
| **M1** | **The 8.45° mast tilt is real and must be modelled.** Not a station-file defect. | Confirmed by the cell photo and, independently, by the taught work object's Z axis matching to 0.02° (§6b). |
| **M2** | **Build on the RobotStudio station placement now**, and treat the taught `wobj_StnN_NoCoord.uframe` as the anchor to reconcile against later. | Orientation is settled; position is good to ~0.1–0.2 m, which is enough to build and exercise both templates. |
| **M3** | **Graft, don't rebuild.** Keep the existing ABB bundle's collision spheres and symmetric `self_collision_ignore`; replace the joint origins and meshes with the RobotStudio-derived values in §4. | The sphere tuning cost a debugging cycle and its 0/300 false-positive result is worth keeping. **But the link positions must be the real robot's (§4), not the ros-industrial dummy's** — so every sphere has to be re-verified against the new geometry, not assumed to carry over. |
| **M4** | **Coordinated motion is the target.** | Stated for the new cell, and the old cell's own RAPID already runs `ufprog:=FALSE, ufmec:="STN1"` (§6b). **D3** stands. |
| **M5** | **The tailstock rides a prismatic joint on the tilt link, locked per project, mapped `not_exported`** so it never reaches `extjoint`. Travel **−2.5 … +2.5 m**, joint origin on the faceplate so the value reads as tube length. | Only option that puts it on the kinematically correct link *and* keeps it a per-project number rather than a per-project bundle edit (§8). Approved 2026-09-01. ⚠ **Two caveats on the range.** It was given as "−2500 mm to +25000 mm"; +25 m is taken as a slipped digit and read as +2500 mm. And the bed only offers **0 … 2.258 m** of usable travel from the faceplate, so the declared range is wider than the hardware in both directions. Harmless — the joint is locked per project — but it lets the UI place the tailstock somewhere impossible. |
| **M6** | **Meshes: metres, binary STL, `*_m.stl`, authored in each link's own frame.** Scale and format converted from the RobotStudio OBJ exports; colours carried from the `.mtl` `Kd` values. | `*_m.stl` is the app's only deterministic unit hint and metres keeps the viewer, Tesseract and cuRobo in agreement with no compensating `scale` attribute. Done — see §7. |
| **M7** | **One URDF link per CAD part**, fixed-jointed to its kinematic parent, so each part keeps its own colour. | The viewer resolves one mesh and one colour per *link name* and ignores later `<visual>` entries, so multi-part links cannot show their real colours any other way (§7). |
| **M8** | **The base → positioner transform comes from `MOC.cfg`'s calibrated `ARM_TYPE.rot_axis_pose_*`**, least-squares fitted, not from the station placement. | It is the controller's own view of where the axes are, so planner and RAPID agree by construction (§6c). |
| **M9** | **`MTD750` is the rotate drive, not a tailstock. There is no tailstock in the ABB CAD** — the M5 prismatic joint carries a synthesized box placeholder. | `MTD750` spans y −1.405…−0.975 against the faceplate at −0.856 and is the small sibling of the `MTD2000` tilt drives. The customer's tailstock is their own tooling. |
| **M10** | **The templates ship without a `weld_workflow_project`.** | Its payload carries `metadata.active_tool_selection`, and every entry in the shipped catalogue names FANUC assets (`fanuc_torch.yaml`, `fanuc_stl`, FANUC nozzle STLs). Authoring one now would bake FANUC asset names into an ABB template. Create it in-app once an ABB tool catalogue exists. |
| **M11** | **Use the CAD Binzel torch for this version**, to be replaced by the Fronius later. | Stated 2026-09-01. Its TCP is derived below rather than inherited from the FANUC bundle. |
| **M12** | **All seven robot links are forced to one off-white** (0.8431 grey, the same the positioner arms carry). | ABB's STEP export gives `LINK3` a red `Kd` — that is the logo/decal colour, not the casting, and it read as a red link in the viewer. Overridden in `COLOUR_OVERRIDE` in `build_meshes.py`; the torch keeps its own colour. |
| **M13** | **Each template carries a `CAD` frame parented straight to the active station's plate link**, holding the faceplate offset, with a `Fixtures` frame under it — Massiv_frame.tgs's exact shape. | "Load CAD" attaches workpieces under the frame named by `ROBODK_WORKPIECE_CAD_MODEL_PARENT_FRAME`, default literal `"CAD"`. Hanging it off the plate link is what makes the workpiece ride **both** the tilt and the rotate/chuck axes. |
| **M14** | **One URDF per station**, not one shared. Each freezes the index axis and the whole idle station into `fixed` joints, baking the angle into the origin. | The app lists every moving URDF joint no mechanism group claims under **"Unassigned Joints"** (`SceneTreeDock`), and one shared URDF left four axes unassigned in every template. Freezing is what **M2** says we do anyway; the idle station keeps all its geometry so it still collides. 10 moving joints per template against the reference project's 9. |
| **M15** | **A zero-range rail**, `linear_rail_joint` with `lower = upper = 0.0` between `base` and `base_link`, in its own `Rail 1` group with a single preset at 0. | MONARC has no rail. `Massiv_frame.tgs` carries exactly this — same zero range — and the Weld Planner's projects are shaped around a rail group, so the structure stays parallel without ever moving the robot. |
| **M19** | **`project_meta.work_zone` = the station number** ("1" / "2"), a text integer per `work_zone_stamp_plan_v1.md` D1/D6. | Lets the HMI tell the two stations' projects apart with one `SELECT`. Station 1 carrying "1" is not a change — D5 says the key is always written and 1 is what the app emits for a project with no weld project. ⚠ **It will not survive the first in-app save.** D4 makes this row a *write-only mirror*; the source of truth is `WeldWorkflowProject.metadata["work_zone"]` (D3), and `ProjectSerializer` re-derives the row from the first weld project on every save. These templates ship without a weld project (**M10**), so a save rewrites Station 2's "2" back to "1". It sticks only once a weld project exists and is stamped via **Tools → Work Zone Settings**. |
| **M18** | **Sphere budget is capped by cuRobo's JIT, not by fidelity: 201 spheres.** Static links are clipped to the 3.360 m reach envelope *and* to the robot side of the rotating wall, which drops all five idle-station bodies. | 918 spheres killed cuRobo with `nvrtc: catastrophic error: out of memory`; the shipped reference runs on 87. The savings buy tighter spheres where they matter — the wall went from 881 mm radii to 324 mm. Quality is gated on a clearance test against the tube + torch envelope, not on raw slack (§10). |
| **M17** | **The tailstock gets its own `custom` mechanism group, NOT a third positioner axis.** | ⚠ Putting it in the positioner group **disabled coordinated motion**, which is the whole point for MONARC. The weld editor takes the positioner's *distal* joint as the coordinated work axis (`work_joint_name = joint_names[-1]`) and asks for its angular travel; `positioner_joint_travel_deg` returns `None` for a prismatic joint by design, so `synchronous_motion_available` was False and the "Synchronous (coordinated) motion" checkbox was greyed out. `custom` also keeps it out of both the Positioner and Rail combos (`_candidate_groups_for_kinds` filters on kind), so the stray "Tailst" row disappears — while the joint stays claimed and out of "Unassigned Joints". |
| **M16** | **Match `Massiv_frame.tgs` on the configuration details**: positioner `shared = 1` / `time_shared`, `default_mechanism_groups = ["Rail 1"]`, tool rows carrying `cog_xyz` / `inertia_*` / `mass_kg`, and a blank `active_external_axis_group_name`. | These were the concrete divergences from the reference project. ⚠ Two things stay deliberately ABB-specific: `motion_role` is `extended_axis` with `embedded_in_pose` (the reference is FANUC `independent_group` / `helper_call`), because **D3** puts the station axes in the arm's own task as `extjoint`; and the slot labels are `eax_b` / `eax_c` from this cell's `MOC.cfg`. |

---

## 12. Open questions

**Closed since v1:** Q1 and Q2 (the calibration was in `MOC.cfg` all along — §6c);
Q4 (Binzel for this version — **M11**); Q5 (travel limits — **M5**); Q6 (meshes are
committed under `bundle/meshes/`).

- **Q3** — Is the new MONARC cell the same layout as this old-cell station, or a new one?
  **D16** treats the old cell as a *configuration* proxy; whether it is a *geometry* proxy is a
  different claim, and §6 leans **entirely** on the old cell's geometry. This is now the single
  largest assumption in the bundle: if the new cell is laid out differently, every number in
  §6c has to be re-read from the new controller's `MOC.cfg` — cheap to redo, but it must be
  redone rather than assumed.
- **Q7** — The TCP is derived from the Binzel CAD: nozzle end at (0.0166, 0, 0.4239) m with the
  neck **45.12°** off `tool0` +Z, plus an assumed **15 mm** stick-out, giving
  `xyz="0.0272 0 0.4344" rpy="0 0.785398 0"`. The 45° is independently corroborated by the
  `Ry(−45°)` in the tooldata already in use. But stick-out is a process number, not a CAD
  number, and **D12** puts `tTG_Weld`'s values on the controller — so the URDF TCP and the
  controller's tooldata must be reconciled before welding or the planner and the robot disagree.
- **Q8** — The real tailstock geometry (**M9**). The placeholder is a 0.30 × 0.25 × 0.45 m box
  straddling the rotation axis on the bed top. Any CAD, or even three dimensions, replaces it.
- **Q9** — Does the ABB `export_v1` block shape match what `AbbTranslator` expects? The
  templates author `eax_b` / `eax_c` explicitly, because with no `abb` block the translator
  derives slots from the mechanism's declared axis order and would land on `eax_a`/`eax_b` —
  silently wrong for this cell. The slot names come from `MOC.cfg`; the *key* semantics are the
  Weld Planner track's to confirm.
- **Q11** — The chuck axis renders as **"Plate"** in the weld editor where the reference
  projects read **"Rot"**. The label is derived from a keyword in the joint name
  (`short_positioner_joint_label`) and is display-only, never persisted, so nothing
  depends on it — but renaming `stnN_plate_joint` to `stnN_rotate_joint` would match
  the house convention. Kept as `plate` for now because that is ABB's own name for the
  axis (`PLATE1`/`PLATE2` in `MOC.cfg`). Say which you prefer.
- **Q10** — Sphere fidelity now that the count is capped at ~190 (§10). Every non-station body
  clears the tube + torch envelope by ≥ 0.34 m, so nothing phantom-blocks the weld — but the
  interchange cover is a large thin shell carrying 882 mm of slack in directions that happen
  not to matter today. If a weld ever needs the torch out past the bed, re-check that margin
  before assuming it still holds.

---

## 13. Unrelated but blocking: `C:` is out of disk

Hit while writing the meshes: `C:` reports **476 GB of 476 GB used, 0 bytes free**, which
truncated the first conversion run mid-write. Clearing ~165 MB of this session's scratch was
enough to finish, but the drive is still effectively full and will keep biting — a truncated
STL fails silently as a malformed mesh rather than as an error, which is exactly the kind of
fault that costs a debugging cycle. Worth clearing properly before the bundle work starts.
`D:` has 289 GB free.
