"""Touch-sense P4 on the MONARC VC: the HMI's C++ ABB touch path serving the planner's module.

    python p4_cpp_touch_vc.py run                      # every cycle below
    python p4_cpp_touch_vc.py run switch-false,unchecked   # a subset, by label

docs/abb_touch_sense_port_v1.md §6 P4; recipe and expected output in robotstudio_setup.md §24.

P5 proved the planner's module with the PROTOTYPE HMI (abb_server.py). P4 puts the real HMI code on
the same rig: `AbbTouchHarness.exe` (TGuideWeldingHMI tests/abb_touch_harness) serves each cycle
with the real `ABBRobot` overrides for ids 16/18/19, the socket-start handshake client, and the
cores RobotCell's cases call - TouchSenseGate, AutoTouchupOffsetCore, StationRebase::ToWire. This
driver reuses P5's cycle, pause handling, TCP trace and judges unchanged
(`planner_touch_vc.planner_cycle` with `hmi_base=CppHmi`), and adds the P4 checks:

  - baseline / shifted / aligned: as P5, and the C++ offset equals the prototype's math on the
    same reports (the two implementations agree), and P5's deltas are reproduced;
  - touch-off (the operator's mode off), switch-absent (no HMI_ABB_TOUCH_SENSE_ENABLED key: the
    default must be off), switch-false: id 16 = 0, the weld runs at the nominal frame;
  - switch-keeps-touchup: the switch off and the weld carrying a stored touch-up - the TSP
    request must NOT clear it (TouchSenseGate::WouldRunAtTsp), and the weld runs on it;
  - unchecked: the weld is not checked to weld, so every weld-frame reply is a skip (status 0,
    Identity) - id 16 must answer 0 (no weld frame served) and the robot never leaves home;
  - rebase: the project's station frame is the reported one moved by a small rigid perturbation,
    the harness adopts the H7 rebase from the handshake's STNFRAME, and the robot must see
    exactly the shifted run's frames - the id-19 reply included, which skipped the rebase
    before P4;
  - miss-retry: the C++ read survives the D3 pause (the robot stops mid-search, Start resumes).

Needs the P5 prerequisites (planner_out/ from `planner_touch_vc.py survey` + `export`, TG_Touch
loaded, PM idle, one station in position) and the harness built (Release x64).
"""
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import touch_probe as tp  # noqa: E402
import planner_touch_vc as pv  # noqa: E402
from touch_search_vc import module_names  # noqa: E402

HMI_ROOT = os.environ.get("TGUIDE_HMI", r"C:\Users\TG_Laptop08\source\repos\TGAlejandroH\TGuideWeldingHMI\TGuideWeldingHMI")
HARNESS = os.path.join(HMI_ROOT, "tests", "abb_touch_harness", "x64", "Release", "AbbTouchHarness.exe")
# The harness links Qt6Core like the application; its DLLs are not next to the exe.
QT_BIN = os.environ.get("TG_QT_BIN", r"C:\Qt\6.4.1\msvc2019_64\bin")
OUT = os.path.join(pv.OUT, "p4")
SWITCH_KEY = "HMI_ABB_TOUCH_SENSE_ENABLED"
# A different calibration of the same station: 11.6 mm and ~1.9 deg, inside the H7 tolerances
# (25 mm / 3 deg), so StationRebase::Decide adopts it. In the station's own frame.
PERTURBATION = [10.0, -5.0, 3.0, 1.0, -0.5, 1.5]
# A stored touch-up on the pendant's 0.005 in grid (0.05 / -0.03 / 0.02 in), so the inches the
# weld-frame reply pushes are exact.
STORED_TOUCHUP_MM = [1.27, -0.762, 0.508]
P5_RUN = os.path.join(pv.OUT, "planner_run_20260926_121106.json")
MATH_TOL = 1e-6      # mm: the C++ and Python offsets from the same reports
REASON_TOUCH = "the weld's touch block runs"
REASON_OFF = "auto touch-ups are off for this run or the weld has no touch points"
REASON_SWITCH = "touch sensing is switched off for this robot brand in config.json"
REASON_NO_FRAME = "the weld's touch-sense weld-frame request served no weld frame (weld skipped or not localized)"
UNCHECKED_LOG = ["10", "5", "4", "16", "4", "100"]


class CppHmi:
    """The C++ harness behind the prototype HMI's interface (what planner_cycle sets and reads)."""

    config_switch = True      # the HMI_ABB_TOUCH_SENSE_ENABLED value; None = the key is absent
    perturbation = None       # project station frame = reported * this (x, y, z, w, p, r)
    stored_touchup_mm = None  # the weld's touch-up before the cycle (base, mm)
    weld_checked = True

    def __init__(self, host="127.0.0.1", port=2000, handshake_port=2001, rws=None, verbose=True):
        self.host, self.port, self.handshake_port = host, port, handshake_port
        self.prog_name = None
        self.tgs_source_dir = None
        self.weld_frame_xyzwpr = None
        self.touch_points_by_weld = None
        self.auto_touchups = False
        self.corrupt_weld_frame = False
        self.request_log = []
        self.last_touch_points = None
        self.touch_offset_base = None
        self.last_touch_frame_xyzwpr = None
        self.cpp_result = None

    def _log(self, msg):
        print("      cpp | " + msg, flush=True)

    def serve_cycle(self):
        if self.corrupt_weld_frame:
            raise RuntimeError("the C++ HMI cannot serve a malformed frame (ABBRobot refuses to format one)")
        (weld, points), = self.touch_points_by_weld.items()
        os.makedirs(OUT, exist_ok=True)
        config = {"ROBOT_1_IP_ADDRESS": self.host, "ROBOT_CONTROLLER_TRANSFER_USERNAME": "Default User",
                  "ROBOT_CONTROLLER_TRANSFER_PASSWORD": "robotics"}
        if self.config_switch is not None:
            config[SWITCH_KEY] = bool(self.config_switch)
        scenario = {"run_port": self.port, "handshake_port": self.handshake_port, "weld_name": weld,
                    "prog_name": self.prog_name,
                    "module_path": os.path.join(self.tgs_source_dir, self.prog_name + ".mod"),
                    "auto_touchups": bool(self.auto_touchups), "weld_frame_xyzwpr": list(self.weld_frame_xyzwpr),
                    "touch_points": [{"nominal": list(p["nominal"]), "snapped_axis": list(p["snapped_axis"])}
                                     for p in points]}
        scenario["weld_checked"] = bool(self.weld_checked)
        if self.stored_touchup_mm is not None:
            scenario["stored_touchup_mm"] = list(self.stored_touchup_mm)
        if self.perturbation is not None:
            scenario["project_station_frame_perturbation"] = list(self.perturbation)
        paths = {k: os.path.join(OUT, "cycle_%s.json" % k) for k in ("config", "scenario", "result")}
        for key, body in (("config", config), ("scenario", scenario)):
            with open(paths[key], "w") as fh:
                json.dump(body, fh, indent=1)
        if os.path.exists(paths["result"]):
            os.replace(paths["result"], paths["result"] + ".prev")
        env = dict(os.environ, PATH=QT_BIN + os.pathsep + os.environ.get("PATH", ""))
        proc = subprocess.Popen([HARNESS, paths["config"], paths["scenario"], paths["result"]], cwd=OUT, env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in iter(proc.stdout.readline, ""):
            line = line.rstrip()
            if line.startswith("serving request "):
                self.request_log.append(line.split()[-1])
                self._log(line)                      # planner_cycle's EventHmi timestamps this
            elif line:
                self._log("  " + line)
        proc.wait()
        if not os.path.exists(paths["result"]):
            raise RuntimeError("the harness wrote no result (exit 0x%X)" % (proc.returncode & 0xFFFFFFFF))
        with open(paths["result"]) as fh:
            self.cpp_result = json.load(fh)
        self.cpp_result["exit_code"] = proc.returncode
        r = self.cpp_result
        self.last_touch_points = r["measured"] or None
        if r["frame19_wire"] is not None:
            self.last_touch_frame_xyzwpr = r["frame19_wire"]
            # The judge composes the offset onto the LIVE nominal: the wire frame's shift from it.
            self.touch_offset_base = [a - b for a, b in zip(r["frame19_wire"][:3], self.weld_frame_xyzwpr[:3])]


def _setup(switch=True, perturbation=None, stored=None, checked=True):
    def apply(hmi):
        hmi.config_switch = switch
        hmi.perturbation = perturbation
        hmi.stored_touchup_mm = stored
        hmi.weld_checked = checked
    return apply


def _to_cad(frame, v):
    # R^T v for the frame's W,P,R rotation: the base-frame vector in the part's axes.
    from abb_server import rotate_by_wpr
    cols = [rotate_by_wpr(frame[3], frame[4], frame[5], e) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    return tuple(sum(c * x for c, x in zip(col, v)) for col in cols)


def _numbers(text):
    import re
    return [float(t) for t in re.findall(r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?", text)]


def _wpr_diff(a, b):
    return max(abs((x - y + 180.0) % 360.0 - 180.0) for x, y in zip(a, b))


def _math_rows(label, r, meta, rows, *, rebased=False):
    """The C++ offset against the prototype's own math on the same reports."""
    from abb_server import auto_touchup_offset, translate_frame
    cpp = r.get("cpp") or {}
    if not r.get("measured") or cpp.get("frame19_wire") is None:
        rows.append(("%s: C++ offset computed" % label, False, "no id-19 frame in the harness result"))
        return
    frame = meta["weld_frame_xyzwpr"]
    points = list(meta["touches"])
    want = translate_frame(frame, auto_touchup_offset(frame, points, r["measured"]))
    got = cpp["frame19_wire"]
    dmm = max(abs(a - b) for a, b in zip(got[:3], want[:3]))
    ddeg = _wpr_diff(got[3:], want[3:])
    rows.append(("%s: the C++ id-19 frame = the prototype's math on the same reports" % label,
                 dmm <= MATH_TOL and ddeg <= 1e-6,
                 "C++ %s vs prototype %s, |dmm| %.1e, |ddeg| %.1e"
                 % (["%.4f" % v for v in got], ["%.4f" % v for v in want], dmm, ddeg)))
    if not rebased:
        dmm = max(abs(a - b) for a, b in zip(cpp["offset_project"], r["offset"]))
        rows.append(("%s: the C++ offset is a pure translation of the served frame" % label, dmm <= MATH_TOL,
                     "offset %s, frame shift %s" % (["%.4f" % v for v in cpp["offset_project"]],
                                                     ["%.4f" % v for v in r["offset"]])))
    rows.append(("%s: id 16 decided by TouchSenseGate: run" % label, cpp.get("gate_reason") == REASON_TOUCH,
                 cpp.get("gate_reason")))


def _nominal_rows(label, r, meta, rows, reason):
    cpp = r.get("cpp") or {}
    ok = r["error"] is None and r["hmi_log"] == pv.TOUCH_OFF_LOG and r["ctl"]["pauses"] == 0
    rows.append(("%s: id 16 = 0 skips the whole block" % label, ok,
                 "%s, pauses %s %s" % (r["hmi_log"], r["ctl"]["pauses"], cpp.get("errors") or "")))
    rows.append(("%s: TouchSenseGate's reason" % label, cpp.get("gate_reason") == reason, cpp.get("gate_reason")))
    trans = r["ctl"]["oframe"][0]
    dmm = max(abs(a - b) for a, b in zip(trans, meta["weld_frame_xyzwpr"][:3]))
    rows.append(("%s: the weld frame stays the served nominal" % label, dmm <= 0.006,
                 "oframe %s, |dmm| %.4f" % (["%.3f" % v for v in trans], dmm)))
    pv._weld_line_check(r, meta, (0.0, 0.0, 0.0), rows, label)


def run(only=None):
    import touch_real_vc as tr
    from touch_search_vc import sym
    if not os.path.exists(HARNESS):
        tp.log("REFUSED: %s is not built (MSBuild AbbTouchHarness.vcxproj, Release x64)" % HARNESS)
        return 1
    if not pv.preflight():
        return 1
    if "TG_Touch" not in module_names():
        tp.log("REFUSED: TG_Touch is not loaded - run touch_search_vc.py (P2 deploy) first")
        return 1
    meta = pv._meta()
    with open(P5_RUN) as fh:
        p5 = json.load(fh)["results"]
    if not tr.load_rig():
        tp.log("RIG LOAD FAILED")
        return 1
    rec = pv._Recorder()
    q = pv.QUANTUM
    plan = [
        ("baseline", (0.0, 0.0, 0.0), {}),
        ("shifted", (3.0, 0.0, -2.0), {}),
        ("aligned", (9 * q, 0.0, -6 * q), {}),
        ("rebase", (3.0, 0.0, -2.0), {"hmi_setup": _setup(perturbation=PERTURBATION)}),
        ("touch-off", (3.0, 0.0, -2.0), {"auto": False}),
        ("switch-absent", (3.0, 0.0, -2.0), {"hmi_setup": _setup(switch=None)}),
        ("switch-false", (3.0, 0.0, -2.0), {"hmi_setup": _setup(switch=False)}),
        ("switch-keeps-touchup", (3.0, 0.0, -2.0), {"hmi_setup": _setup(switch=False, stored=STORED_TOUCHUP_MM)}),
        ("unchecked", (3.0, 0.0, -2.0), {"hmi_setup": _setup(checked=False)}),
        ("miss-retry", (0.0, 0.0, 0.0), {"miss_first": True}),
    ]
    if only:
        unknown = sorted(set(only) - {label for label, _, _ in plan})
        if unknown:
            tp.log("REFUSED: unknown cycle label(s) %s" % unknown)
            return 1
        plan = [step for step in plan if step[0] in only]
    rows, results, deltas = [], {}, {}
    try:
        for i, (label, shift, opts) in enumerate(plan):
            if i:
                time.sleep(pv.PAUSE_S)
            r = pv.planner_cycle(label, shift, rec, hmi_base=CppHmi, **opts)
            results[label] = r
            cpp = r.get("cpp") or {}
            rows.append(("%s: the harness served the cycle without an error" % label,
                         cpp.get("exit_code") == 0 and not cpp.get("errors"), "%s" % (cpp.get("errors") or "")))
            if label in ("baseline", "shifted", "aligned", "rebase", "miss-retry"):
                judged = dict(r, ctl=dict(r["ctl"], pauses=0)) if label == "miss-retry" else r
                deltas[label] = pv._judge_touch(label, shift, judged, rows, pv.PLANNER_LOG)
                if deltas[label] is not None:
                    r["weld_line"] = pv._weld_line_check(r, meta, deltas[label], rows, label)
                    r["searches"] = pv._search_measurements(r, meta)
                _math_rows(label, r, meta, rows, rebased=(label == "rebase"))
                rows.append(("%s: the TSP request cleared the touch-up before measuring" % label,
                             cpp.get("tsp_clear") == "cleared", cpp.get("tsp_clear")))
            elif label == "touch-off":
                _nominal_rows(label, r, meta, rows, REASON_OFF)
            elif label in ("switch-absent", "switch-false"):
                _nominal_rows(label, r, meta, rows, REASON_SWITCH)
            elif label == "switch-keeps-touchup":
                from abb_server import translate_frame
                ok = r["error"] is None and r["hmi_log"] == pv.TOUCH_OFF_LOG and r["ctl"]["pauses"] == 0
                rows.append(("%s: id 16 = 0 skips the whole block" % label, ok, "%s" % r["hmi_log"]))
                rows.append(("%s: TouchSenseGate's reason" % label, cpp.get("gate_reason") == REASON_SWITCH,
                             cpp.get("gate_reason")))
                rows.append(("%s: the TSP request kept the stored touch-up" % label,
                             cpp.get("tsp_clear", "").startswith("kept")
                             and max(abs(a - b) for a, b in zip(cpp.get("stored_touchup_after") or [9e9] * 3,
                                                                STORED_TOUCHUP_MM)) < 1e-9,
                             "%s, stored after the cycle %s" % (cpp.get("tsp_clear"), cpp.get("stored_touchup_after"))))
                want = translate_frame(meta["weld_frame_xyzwpr"], STORED_TOUCHUP_MM)
                trans = r["ctl"]["oframe"][0]
                dmm = max(abs(a - b) for a, b in zip(trans, want[:3]))
                rows.append(("%s: the weld frame = the nominal with the stored touch-up" % label, dmm <= 0.006,
                             "oframe %s, want %s, |dmm| %.4f" % (["%.3f" % v for v in trans],
                                                                 ["%.3f" % v for v in want[:3]], dmm)))
                pushed = _numbers(sym("TG_Comms", "posTG_Touchup"))
                want_in = [v / 25.4 for v in STORED_TOUCHUP_MM]
                din = max(abs(a - b) for a, b in zip(pushed, want_in)) if len(pushed) == 3 else 9e9
                rows.append(("%s: the pendant got the stored touch-up in inches" % label, din <= 1e-4,
                             "posTG_Touchup %s, want %s" % (pushed, ["%.4f" % v for v in want_in])))
                r["weld_line"] = pv._weld_line_check(r, meta, _to_cad(meta["weld_frame_xyzwpr"], STORED_TOUCHUP_MM),
                                                     rows, label)
            elif label == "unchecked":
                ok = r["error"] is None and r["hmi_log"] == UNCHECKED_LOG and r["ctl"]["pauses"] == 0
                rows.append(("%s: the skipped weld's block and weld are both skipped" % label, ok,
                             "%s (want %s)" % (r["hmi_log"], UNCHECKED_LOG)))
                rows.append(("%s: TouchSenseGate's reason: no weld frame served" % label,
                             cpp.get("gate_reason") == REASON_NO_FRAME, cpp.get("gate_reason")))
                moved = [s for s in r["samples"] if abs(s[3] - r["samples"][0][3]) > 5.0]
                rows.append(("%s: the robot never left home" % label, bool(r["samples"]) and not moved,
                             "%d samples more than 5 mm from the start height" % len(moved)))
            if label == "rebase":
                rows.append(("rebase: StationRebase adopted the perturbed station frame",
                             cpp.get("rebase", "").startswith("adopted"), cpp.get("rebase")))
                shifted = (results.get("shifted") or {}).get("cpp") or {}
                if cpp.get("frame19_wire") and shifted.get("frame19_wire"):
                    dmm = max(abs(a - b) for a, b in zip(cpp["frame19_wire"][:3], shifted["frame19_wire"][:3]))
                    ddeg = _wpr_diff(cpp["frame19_wire"][3:], shifted["frame19_wire"][3:])
                    rows.append(("rebase: the id-19 frame on the wire = the shifted run's",
                                 dmm <= q + 0.03 and ddeg <= 1e-6,
                                 "|dmm| %.4f (need <= %.2f: the same block, one rig quantum apart at most), |ddeg| %.1e"
                                 % (dmm, q + 0.03, ddeg)))
                    project = cpp["weld_frame_project"]
                    moved = max(abs(a - b) for a, b in zip(project[:3], meta["weld_frame_xyzwpr"][:3]))
                    unrebased = max(abs(a - b) for a, b in zip(cpp["frame19_unrebased"][:3], cpp["frame19_wire"][:3]))
                    rows.append(("rebase: without the P4 fix the id-19 frame would have been off", unrebased > 1.0,
                                 "project-world nominal %.1f mm from the live one; the unrebased id-19 frame "
                                 "%.2f mm from the one sent" % (moved, unrebased)))
            if label == "miss-retry":
                p = r["pause"] or {}
                rows.append(("miss-retry: the first search missed and paused (D3, reason 3)",
                             p.get("reason") == 3.0 and p.get("reports_before") == 0, "pause %s" % p))
                rows.append(("miss-retry: exactly one pause, the C++ read survived it in lockstep",
                             r["ctl"]["pauses"] == 1 and r["hmi_log"] == pv.PLANNER_LOG,
                             "pauses %s, log %s" % (r["ctl"]["pauses"], r["hmi_log"])))
        if deltas.get("baseline") and deltas.get("aligned"):
            diff = [a - b for a, b in zip(deltas["aligned"], deltas["baseline"])]
            err = max(abs(a - b) for a, b in zip(diff, {l: s for l, s, _ in plan}["aligned"]))
            rows.append(("aligned - baseline = the aligned shift", err <= 0.03,
                         "%s, max err %.4f (need <= 0.030)" % (["%.3f" % v for v in diff], err)))
        for label in ("baseline", "shifted", "aligned", "miss-retry"):
            mine, theirs = results.get(label) or {}, p5.get(label) or {}
            if mine.get("offset") and theirs.get("offset"):
                err = max(abs(a - b) for a, b in zip(mine["offset"], theirs["offset"]))
                rows.append(("%s: the C++ HMI measures what the prototype measured in P5" % label, err <= 0.03,
                             "C++ %s vs P5 %s, max diff %.4f"
                             % (["%.3f" % v for v in mine["offset"]], ["%.3f" % v for v in theirs["offset"]], err)))
    finally:
        out = os.path.join(OUT, time.strftime("p4_run_%Y%m%d_%H%M%S.json"))
        os.makedirs(OUT, exist_ok=True)
        with open(out, "w") as fh:
            json.dump({"results": results, "rows": rows}, fh, indent=1, default=str)
        tp.log("raw results: %s" % out)
        for step in (lambda: tr.run_quick("TG_TrRestore"), lambda: tr.run_quick("TG_TrHome"), tr.unload_rig,
                     tp.back_to_pm):
            try:
                step()
            except Exception as exc:
                tp.log("CLEAN-UP STEP FAILED: %s - finish by hand (TG_TrRestore, park, unload TG_TrRig, PP to main)"
                       % exc)
        names = {k: sym("TG_Cell", k) for k in ("stTG_TouchDI", "stTG_TouchOnDO", "stTG_TouchActiveDI")}
        rows.append(("TG_Cell welder names restored", names == {"stTG_TouchDI": '"diWld1Touched"',
                     "stTG_TouchOnDO": '"doWld1TouchOn"', "stTG_TouchActiveDI": '"diWld1TouchActive"'}, "%s" % names))
    tp.log("")
    for label in ("baseline", "shifted", "aligned", "rebase"):
        for m in (results.get(label) or {}).get("searches") or []:
            tp.log("measure %-8s touch %s" % (label, m))
        wl = (results.get(label) or {}).get("weld_line")
        if wl:
            tp.log("measure %-8s weld pass %s" % (label, wl))
    for name, ok, detail in rows:
        tp.log("%s  %s -- %s" % ("PASS" if ok else "FAIL", name, detail))
    passed = sum(1 for _, ok, _ in rows if ok)
    tp.log("%d/%d PASS" % (passed, len(rows)))
    return 0 if rows and passed == len(rows) else 1


def main(argv):
    if (argv[0] if argv else "run") != "run":
        raise SystemExit("usage: p4_cpp_touch_vc.py run [label,label,...]")
    return run(argv[1].split(",") if len(argv) > 1 else None)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
