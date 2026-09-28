"""Touch-sense P5: the Weld Planner's own ABB export of TD05Touch's touch-sensed weld.

    python planner_touch_export.py [survey.json] [out_dir]

Called by ``planner_touch_vc.py export``; recipe in robotstudio_setup.md §23.

Builds the program the planner would ship for TD05Touch's part (docs/abb_touch_sense_port_v1.md
§6 P3: a block in the served frame [1600,0,1450 | Rz 90], touched on its -X face and its top
face, then welded 5 mm over the top) and compiles it with the planner's own code, read-only:

- the touch family through ``WeldTouchGenerationService`` itself: ``_build_bridge_programs``,
  ``_build_touch_program`` and ``_build_wrapper_program``, with the ABB blocks provider and the
  export mode resolved the way ``_resolve_touch_seed`` resolves it;
- the weld in the planner's HMI shape, framed by ``AbbProgramBlocksProvider``'s weld blocks;
- a package main framed by its package blocks (header, TG_WeldPrep, footer + abort_end);
- one flattened module compiled by ``ProgramCompiler`` + ``AbbTranslator`` under the ABB HMI Mode
  profile, with the MONARC station positioner modelled as the app authors it (STN1, tilt ->
  eax_b, rotate -> eax_c), so every robtarget carries the station's axes.

What stands in for the planner's solvers, and why that is fair: the touch solve (IK, torch
orientation sweep) and the bridge planner (cuRobo) produce joint maps and poses; here they come
from ``TG_P5Survey.mod``, which measured on the controller the joint solution the robot takes at
each pose. The FK the builders ask for is answered from the same survey. Everything P5 changed
- the blocks, the brand-native marking, the compiler's contact hand-off, the translator's
search - runs as it would in the app.

Also writes the touch points into ``<prog>.tgs`` exactly as the planner persists them
(``weld_touch_points``, ``normalize_touch_point_payload`` + the root-CAD canonicalization), for the
prototype HMI to read the way the real HMI reads a project.
"""
import json
import math
import os
import sqlite3
import sys
import types

sys.dont_write_bytecode = True             # never write into curobo_suite
HERE = os.path.dirname(os.path.abspath(__file__))
CUROBO = os.environ.get("CUROBO_SUITE", os.path.normpath(os.path.join(HERE, "..", "..", "..", "curobo_suite")))
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, CUROBO)

from tguide_lib._tesseract_dll_preload import ensure_msvc_runtime_loaded  # noqa: E402

ensure_msvc_runtime_loaded()

import numpy as np  # noqa: E402

from abb_server import TOUCH_REAL_DEMO, euler_wpr_to_quat  # noqa: E402
from tguide_lib.ExternalAxisGroup import ExternalAxisGroup  # noqa: E402
from tguide_lib.ProgramCompiler import ProgramCompiler  # noqa: E402
from tguide_lib.ProgramInstruction import (  # noqa: E402
    INSTR_CALL_PROGRAM,
    INSTR_CUSTOM_CODE,
    INSTR_MOVE_J,
    INSTR_MOVE_L,
    INSTR_PROCESS_END,
    INSTR_PROCESS_START,
    INSTR_SET_FRAME,
    INSTR_SET_ROUNDING,
    INSTR_SET_SPEED,
    INSTR_SET_TCP,
    EmbeddedTargetPose,
    ProgramInstruction,
)
from tguide_lib.RobotGroup import RobotGroup  # noqa: E402
from tguide_lib.RobotProgram import RobotProgram  # noqa: E402
from tguide_lib.Translators.AbbTranslator import AbbTranslator  # noqa: E402
from tguide_lib.WeldWorkflowProject import WeldItem  # noqa: E402
from tguide_lib.robot_brand_program_blocks.abb_program_blocks import AbbProgramBlocksProvider  # noqa: E402
from tguide_lib.robot_brand_program_blocks.base_program_blocks import (  # noqa: E402
    WeldProgramBlocksContext,
    WorkflowPackageProgramBlocksContext,
)
from tguide_lib.robot_branding import ROBOT_BRAND_ABB  # noqa: E402
from tguide_lib.workflows.abb_output_profiles import (  # noqa: E402
    ABB_EXPORT_MODE_HMI,
    ABB_OUTPUT_PROFILE_ID_HMI,
    build_abb_program_export_config,
    resolve_abb_output_profile,
)
from tguide_lib.workflows.weld_planner_touch import normalize_touch_point_payload  # noqa: E402
from tguide_lib.workflows.weld_touch_generation_native import WeldTouchGenerationService  # noqa: E402

PROG_NAME = "TsPlan05"
WELD_NAME = "Weld2"
CAD_FRAME = "CAD"
TCP_NAME = "tcp_torch"
ARM = [f"joint_{i}" for i in range(1, 7)]
TILT, ROT = "stn1_tilt_joint", "stn1_rotate_joint"
POSITIONER_GROUP = "Station 1"
# TD05Touch's torch orientation, in the served frame (part coordinates).
TOOL_Q = (0.353554, 0.612372, 0.612372, -0.353554)
# TD05Touch's poses (part mm) = the survey's pose names.
POSES_MM = {
    "Via": (-30.0, 0.0, 40.0),
    "WA": (50.0, -30.0, 40.0), "WS": (50.0, -30.0, 5.0),
    "WE": (50.0, 30.0, 5.0), "WD": (50.0, 30.0, 40.0),
}
TOUCH_POSES = (("A1", "C1"), ("A2", "C2"))      # (approach, contact) per touch, in touch order
APPROACH_SPEED_M_S = 0.775                     # FANUC's 775 mm/sec, the planner default
WELD_SPEED_M_S = 0.00889                       # 21 IPM, the HMI seed default (welddata governs)


def _quat_matrix(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def frame_world():
    """The served weld frame (TOUCH_REAL_DEMO) as the planner's CAD frame in world, metres."""
    f = TOUCH_REAL_DEMO["weld_frame_xyzwpr"]
    m = np.eye(4)
    m[:3, :3] = _quat_matrix(*euler_wpr_to_quat(*f[3:6]))
    m[:3, 3] = np.asarray(f[:3], dtype=float) * 0.001
    return m


def pose_world(xyz_mm):
    local = np.eye(4)
    local[:3, :3] = _quat_matrix(*TOOL_Q)
    local[:3, 3] = np.asarray(xyz_mm, dtype=float) * 0.001
    return frame_world() @ local


def touch_geometry():
    """Per touch: nominal, snapped axis, approach (touch + standoff * axis) in part mm."""
    out = []
    standoff = float(TOUCH_REAL_DEMO["standoff"])
    for point in TOUCH_REAL_DEMO["touch_points"]:
        nominal = tuple(float(v) for v in point["nominal"])
        axis = tuple(float(v) for v in point["snapped_axis"])
        out.append({"nominal": nominal, "snapped_axis": axis,
                    "approach": tuple(n + standoff * a for n, a in zip(nominal, axis))})
    return out


class _Scene:
    """The part of a scene tree the compiler reaches for: the robot, the station, the frame."""

    def __init__(self, programs, station_name):
        self._programs = {str(p.program_id): p for p in programs}
        self.robot_groups = [RobotGroup(name="Robot 1", joint_names=list(ARM), arm_joint_names=list(ARM),
                                        robot_brand=ROBOT_BRAND_ABB)]
        # The app's D4 dialog shape for a MONARC station (tests/test_abb_indexed_station_activation.py).
        self._station = ExternalAxisGroup(
            name=POSITIONER_GROUP, joint_names=[TILT, ROT], kind="positioner",
            attached_robot_groups=["Robot 1"], positioner_base_link="stn1_base",
            positioner_ee_link="stn1_plate", workpiece_frame_name=CAD_FRAME,
            metadata={"export_v1": {"motion_role": "independent_group", "brands": {"abb": {
                "controller_group": station_name,
                "axes": [
                    {"joint_name": TILT, "label": "eax_b", "unit": "deg", "export": "embedded_in_pose"},
                    {"joint_name": ROT, "label": "eax_c", "unit": "deg", "export": "embedded_in_pose"},
                ]}}}},
        )
        self._tool = np.eye(4)
        self._tool[2, 3] = 0.2                  # tTG_Weld's resident TCP on the MONARC VC

    def get_robot_program(self, program_id):
        return self._programs.get(str(program_id))

    def get_cell_program(self, program_id):
        return None

    def get_robot_group(self, name):
        return self.robot_groups[0] if str(name) == "Robot 1" else None

    def get_robot_tool(self, name):
        if str(name) != TCP_NAME:
            return None
        return types.SimpleNamespace(name=TCP_NAME, robot_group_name="Robot 1", tool_matrix=lambda: self._tool)

    def get_robot_target(self, name):
        return None

    def get_external_axis_group(self, name):
        return self._station if str(name) == POSITIONER_GROUP else None

    def get_all_external_axis_groups(self):
        return [self._station]

    def get_frame_by_name(self, name):
        return object() if str(name) == CAD_FRAME else None


class _Kinematics:
    """The robot's FK/IK for the builders, answered from the controller survey."""

    def __init__(self, survey):
        self.station_axes_deg = survey["poses"]["Home"]["extax"][1:3]   # eax_b, eax_c
        self.joint_maps, self.poses = {}, {}
        geometry = dict(POSES_MM)
        for touch, (a, c) in zip(touch_geometry(), TOUCH_POSES):
            geometry[a], geometry[c] = touch["approach"], touch["nominal"]
        for name, xyz in geometry.items():
            robax = survey["poses"][name]["robax_deg"]
            joint_map = {j: math.radians(v) for j, v in zip(ARM, robax)}
            joint_map[TILT] = math.radians(self.station_axes_deg[0])
            joint_map[ROT] = math.radians(self.station_axes_deg[1])
            self.joint_maps[name] = joint_map
            self.poses[name] = pose_world(xyz)
        home = {j: math.radians(v) for j, v in zip(ARM, survey["poses"]["Home"]["robax_deg"])}
        home[TILT], home[ROT] = self.joint_maps["Via"][TILT], self.joint_maps["Via"][ROT]
        self.joint_maps["Home"] = home
        # Home is a joint posture (MoveAbsJ); its pose is only carried along. The TCP there, as
        # RWS reports it on the MONARC VC: [1652.3, 0, 1622.5] mm, q [0.5, 0, 0.866025, 0].
        home_pose = np.eye(4)
        home_pose[:3, :3] = _quat_matrix(0.5, 0.0, 0.866025, 0.0)
        home_pose[:3, 3] = [1.6523, 0.0, 1.6225]
        self.poses["Home"] = home_pose

    def _name_of(self, joint_map):
        for name, known in self.joint_maps.items():
            if all(abs(float(joint_map.get(j, 1e9)) - known[j]) < 1e-9 for j in ARM):
                return name
        raise KeyError("no surveyed pose for joint map %r" % (joint_map,))

    def world_tcp_from_joint_map(self, joint_map, tool_matrix=None):
        del tool_matrix
        return self.poses[self._name_of(joint_map)]


class _BridgePlanner:
    """The bridge planner's answer for each leg: TD05Touch's own path (via the point above touch 1)."""

    def __init__(self, kin):
        self._kin = kin
        self.calls = []

    def plan_bridge_joint_maps(self, **kwargs):
        self.calls.append((kwargs.get("from_program_name"), kwargs.get("to_program_name")))
        start, goal = kwargs["start_joint_map"], kwargs["goal_joint_map"]
        names = (self._kin._name_of(start), self._kin._name_of(goal))
        # weld approach -> touch 1 and touch 1 -> touch 2 pass over "Via", as TD05Touch does.
        via = [self._kin.joint_maps["Via"]] if names in (("WA", "A1"), ("A1", "A2")) else []
        return types.SimpleNamespace(success=True, waypoint_joint_maps=via, error_text="")

    def audit_joint_samples(self, **kwargs):
        return types.SimpleNamespace(collision_free=True, error_text="", first_hit=None)


def _embedded(kin, name, cartesian=False, joints=True):
    local = np.linalg.inv(frame_world()) @ kin.poses[name]
    jm = kin.joint_maps[name]
    return EmbeddedTargetPose(
        parent_frame_name=CAD_FRAME,
        cartesian_matrix_local_flat=[float(v) for v in local.reshape(-1)],
        joint_values={j: jm[j] for j in ARM} if joints else {},
        external_joint_values={TILT: jm[TILT], ROT: jm[ROT]},
        has_external_snapshot=True,
        export_movej_as_cartesian=bool(cartesian),
    )


def _code(program_id, lines, prefix):
    return [ProgramInstruction(program_id=program_id, instruction_id=f"{prefix}{i}", instr_type=INSTR_CUSTOM_CODE,
                               custom_code=line, brand_native_code=True)
            for i, line in enumerate(str(lines or "").splitlines() if isinstance(lines, str) else lines)
            if str(line).strip() != ""]


def _instr(program_id, iid, kind, **kw):
    return ProgramInstruction(program_id=program_id, instruction_id=iid, instr_type=kind, **kw)


def build_weld_program(kin):
    """The weld in the planner's HMI shape (tests/test_abb_hmi_program_blocks.py `_weld_program`,
    the generated `PPP_*` programs): blocks' program_start, frame, a Cartesian approach that rides
    the served frame, the arc span framed by the blocks' path_start/path_finish, the depart."""
    events = AbbProgramBlocksProvider().build_weld_program_blocks(WeldProgramBlocksContext(
        weld_name=WELD_NAME, subroutine_name="P" + WELD_NAME, weld_mode="library", procedure_number=1,
        schedule_number=1, weave_type="", process_point_count=2, is_full_circle=False,
        export_mode=ABB_EXPORT_MODE_HMI)).program_events
    pid = "weld_program_" + WELD_NAME
    ins = _code(pid, events.program_start, "ps_")
    ins += [
        _instr(pid, "frame", INSTR_SET_FRAME, frame_name=CAD_FRAME),
        _instr(pid, "tcp", INSTR_SET_TCP, tcp_name=TCP_NAME),
        _instr(pid, "vj", INSTR_SET_SPEED, joint_speed_rad_s=math.radians(100.0)),
        _instr(pid, "fine", INSTR_SET_ROUNDING, rounding_mode="fine", rounding_m=0.0),
        _instr(pid, "approach", INSTR_MOVE_J, embedded_target=_embedded(kin, "WA", cartesian=True)),
        _instr(pid, "vl", INSTR_SET_SPEED, linear_speed_m_s=APPROACH_SPEED_M_S),
        # The seam's first process point is its start: ArcLStart there, ArcLEnd at its end.
        _instr(pid, "arc_on", INSTR_PROCESS_START, custom_code=events.path_start, brand_native_code=True),
        _instr(pid, "start", INSTR_MOVE_L, embedded_target=_embedded(kin, "WS")),
        _instr(pid, "vw", INSTR_SET_SPEED, linear_speed_m_s=WELD_SPEED_M_S),
        _instr(pid, "seam", INSTR_MOVE_L, embedded_target=_embedded(kin, "WE")),
        _instr(pid, "arc_off", INSTR_PROCESS_END, custom_code=events.path_finish, brand_native_code=True),
        _instr(pid, "vl2", INSTR_SET_SPEED, linear_speed_m_s=APPROACH_SPEED_M_S),
        _instr(pid, "depart", INSTR_MOVE_L, embedded_target=_embedded(kin, "WD")),
    ]
    ins += _code(pid, events.program_finish, "pf_")
    return RobotProgram(program_id=pid, name="P" + WELD_NAME, parent_frame_name=CAD_FRAME,
                        robot_group_name="Robot 1", robot_brand=ROBOT_BRAND_ABB, instructions=ins)


def build_home_program(kin, pid):
    return RobotProgram(program_id=pid, name=pid, parent_frame_name=CAD_FRAME, robot_group_name="Robot 1",
                        robot_brand=ROBOT_BRAND_ABB, instructions=[
                            _instr(pid, "frame", INSTR_SET_FRAME, frame_name=CAD_FRAME),
                            _instr(pid, "vj", INSTR_SET_SPEED, joint_speed_rad_s=math.radians(100.0)),
                            _instr(pid, "fine", INSTR_SET_ROUNDING, rounding_mode="fine", rounding_m=0.0),
                            _instr(pid, "home", INSTR_MOVE_J, embedded_target=_embedded(kin, "Home")),
                        ])


def build_touch_family(kin, teach_mode=True):
    """The planner's touch programs for WELD_NAME, from WeldTouchGenerationService's own builders."""
    main_window = types.SimpleNamespace(
        config={"WELDER_ENABLE_FRONIUS_TEACH_MODE": bool(teach_mode)},
        scene_tree=None,
        path_planning_controller=None,
        program_controller=types.SimpleNamespace(_world_tcp_from_joint_map=kin.world_tcp_from_joint_map),
        # Only its joint list is read here (bridge endpoint de-duplication, `_joint_distance`).
        robot_urdf=types.SimpleNamespace(urdf_actuated_joint_names=list(ARM) + [TILT, ROT]),
        _joint_names_for_group=lambda group_name, fallback_all=False: list(ARM),
        _infer_robot_configuration=lambda joint_map: ("Front", "Up", "No Flip"),
    )
    planner = _BridgePlanner(kin)
    main_window.path_planning_controller = planner
    service = WeldTouchGenerationService(main_window)
    # The two app-state lookups a bridge makes that a headless run has no scene for: endpoint
    # normalization and other robot groups' lanes (there are none on this cell).
    service._normalized_bridge_endpoint_joint_maps = (
        lambda *, seed, start_joint_map, goal_joint_map, scene_snapshot=None:
        (dict(start_joint_map), dict(goal_joint_map), list(ARM)))
    service._bridge_program_joint_map = lambda *, seed, joint_map, scene_snapshot=None: dict(joint_map)
    weld = WeldItem(weld_id="weld_2", name=WELD_NAME)
    seed = types.SimpleNamespace(
        frame_name=CAD_FRAME, frame_world=frame_world(), weld_program_id="weld_program_" + WELD_NAME,
        weld_program_name="P" + WELD_NAME, robot_group_name="Robot 1", robot_brand=ROBOT_BRAND_ABB,
        tcp_name=TCP_NAME, weld_start_joint_map=kin.joint_maps["WA"],
        # What `_resolve_touch_seed` resolves for an indexed HMI weld (`_abb_touch_export_mode`).
        export_mode=ABB_EXPORT_MODE_HMI,
    )
    solved = []
    for index, (touch, (a, c)) in enumerate(zip(touch_geometry(), TOUCH_POSES)):
        solved.append(types.SimpleNamespace(
            touch_index=index,
            touch_program_id=f"touch_program_weld_2_{index}",
            touch_program_name=service._touch_program_name(weld, index),
            search_axis_label=service._legacy_search_axis_label(touch["snapped_axis"]),
            solve_mode="strict",
            approach_pose_world=kin.poses[a], contact_pose_world=kin.poses[c],
            approach_joint_map=kin.joint_maps[a], contact_joint_map=kin.joint_maps[c],
        ))
    bridges = service._build_bridge_programs(weld=weld, seed=seed, solved_touch_points=solved)
    touches = [service._build_touch_program(weld=weld, seed=seed, solved_touch=s) for s in solved]
    wrapper = service._build_wrapper_program(weld=weld, seed=seed, solved_touch_points=solved,
                                             bridge_programs=bridges)
    return wrapper, touches + bridges, planner.calls, solved


def build_main(programs_in_order):
    """The package main: header, TG_WeldPrep, the components in order, footer (abort_end)."""
    blocks = AbbProgramBlocksProvider().build_workflow_package_program_blocks(WorkflowPackageProgramBlocksContext(
        project_name="P5 touch demo", project_password=PROG_NAME, main_program_name=PROG_NAME,
        robot_brand=ROBOT_BRAND_ABB, has_global_capture=False, add_start_home_call=False,
        export_mode=ABB_EXPORT_MODE_HMI))
    pid = "main"
    ins = _code(pid, blocks.main_start_lines, "h") + _code(pid, blocks.before_first_component_lines, "b")
    for index, (comment, program) in enumerate(programs_in_order):
        if comment:
            ins += _code(pid, ["! " + comment], f"c{index}_")
        ins.append(_instr(pid, f"call{index}", INSTR_CALL_PROGRAM, called_program_id=program.program_id,
                          name=program.name))
    ins += _code(pid, blocks.main_finish_lines, "f")
    return RobotProgram(program_id=pid, name=PROG_NAME, parent_frame_name=CAD_FRAME, robot_group_name="Robot 1",
                        robot_brand=ROBOT_BRAND_ABB, instructions=ins)


def write_tgs(path, solved):
    """weld_items + weld_touch_points as the planner persists them (ProjectSchema / ProjectSerializer)."""
    if os.path.exists(path):
        os.remove(path)
    db = sqlite3.connect(path)
    try:
        db.executescript("""
            CREATE TABLE weld_items(weld_id TEXT PRIMARY KEY, project_node_id TEXT NOT NULL, name TEXT NOT NULL,
              robot_group_name TEXT NOT NULL DEFAULT '', order_index INTEGER NOT NULL DEFAULT 0,
              payload_json TEXT NOT NULL DEFAULT '{}', generation_state_json TEXT NOT NULL DEFAULT '{}');
            CREATE TABLE weld_touch_points(touch_point_id TEXT PRIMARY KEY, weld_id TEXT NOT NULL,
              order_index INTEGER NOT NULL DEFAULT 0, payload_json TEXT NOT NULL DEFAULT '{}');
        """)
        db.execute("INSERT INTO weld_items VALUES (?,?,?,?,?,?,?)",
                   ("weld_2", "weld_project_1", WELD_NAME, "Robot 1", 0, "{}", "{}"))
        for index, touch in enumerate(touch_geometry()):
            payload = normalize_touch_point_payload({
                "placement_mode": "manual",
                "touch_point_cad_local_mm": list(touch["nominal"]),
                "approach_point_cad_local_mm": list(touch["approach"]),
                "base_normal_cad_local": list(touch["snapped_axis"]),
                "normal_cad_local": list(touch["snapped_axis"]),
                "snapped_axis_cad_local": list(touch["snapped_axis"]),
            })
            payload["coordinate_frame"] = "root_cad"       # ProjectSerializer._canonicalize_weld_touch_payload
            db.execute("INSERT INTO weld_touch_points VALUES (?,?,?,?)",
                       (f"touch_{index}", "weld_2", index, json.dumps(payload, sort_keys=True)))
        db.commit()
    finally:
        db.close()


def export(survey_path, out_dir, teach_mode=True):
    with open(survey_path) as fh:
        survey = json.load(fh)
    kin = _Kinematics(survey)
    wrapper, family, bridge_calls, solved = build_touch_family(kin, teach_mode=teach_mode)
    weld = build_weld_program(kin)
    home_start, home_end = build_home_program(kin, "home_start"), build_home_program(kin, "home_end")
    main = build_main([("", home_start), ("TouchSense for " + WELD_NAME, wrapper),
                       ("Start of " + WELD_NAME, weld), ("", home_end)])
    programs = [main, wrapper, weld, home_start, home_end] + list(family)
    config = build_abb_program_export_config(main, resolve_abb_output_profile(ABB_OUTPUT_PROFILE_ID_HMI))
    compiler = ProgramCompiler(
        scene_tree=_Scene(programs, "STN%d" % int(survey.get("station", 1))),
        get_target_cartesian_world_matrix_fn=lambda _name: None,
        get_frame_world_matrix_fn=lambda name: frame_world() if str(name) == CAD_FRAME else None,
        resolve_program_export_config_fn=lambda _program: dict(config),
        get_joint_unit_fn=lambda _name: "rad",
        get_link_world_matrix_fn=lambda _name: None,
    )
    translator = AbbTranslator("R")
    ok, message = compiler.compile(main.program_id, translator, None, inline_called_programs=True)
    if not ok:
        raise SystemExit("planner compile failed: %s" % message)
    text = translator.get_program_string()
    os.makedirs(out_dir, exist_ok=True)
    mod_path = os.path.join(out_dir, PROG_NAME + ".mod")
    with open(mod_path, "w", newline="\r\n") as fh:
        fh.write(text if text.endswith("\n") else text + "\n")
    tgs_path = os.path.join(out_dir, PROG_NAME + ".tgs")
    write_tgs(tgs_path, solved)
    meta = {
        "prog_name": PROG_NAME, "weld_name": WELD_NAME, "module": mod_path, "tgs": tgs_path,
        "weld_frame_xyzwpr": list(TOUCH_REAL_DEMO["weld_frame_xyzwpr"]),
        "station_axes_deg": kin.station_axes_deg,
        "touches": [{"nominal": t["nominal"], "snapped_axis": t["snapped_axis"], "approach": t["approach"],
                     "search_axis_label": s.search_axis_label} for t, s in zip(touch_geometry(), solved)],
        "weld_line_mm": [POSES_MM["WS"], POSES_MM["WE"]],
        "bridge_legs": bridge_calls,
        "warnings": list(translator.export_warnings),
        "curobo_suite": CUROBO,
    }
    with open(os.path.join(out_dir, PROG_NAME + "_export.json"), "w") as fh:
        json.dump(meta, fh, indent=1)
    return meta


def main(argv):
    survey_path = argv[0] if len(argv) > 0 else os.path.join(HERE, "planner_out", "survey.json")
    out_dir = argv[1] if len(argv) > 1 else os.path.join(HERE, "planner_out")
    meta = export(survey_path, out_dir)
    print("wrote %s and %s" % (meta["module"], meta["tgs"]))
    print("bridge legs planned: %s" % meta["bridge_legs"])
    for warning in meta["warnings"]:
        print("  warning: " + warning[:160])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
