"""Touch-sense P5 on the MONARC VC: the Weld Planner's OWN ABB export of a touch-sensed weld.

    python planner_touch_vc.py survey    # joints + station axes at TD05Touch's poses
    python planner_touch_vc.py export    # the planner export -> planner_out/
    python planner_touch_vc.py run       # the experiments (below), against the rig
    python planner_touch_vc.py all       # the three in order

docs/abb_touch_sense_port_v1.md §6 P5; recipe and expected output in robotstudio_setup.md §23.

P3 proved the RAPID side with a HAND-WRITTEN module, `TGS/TD05Touch.mod`. P5 replaces it with
what the Weld Planner (curobo_suite) emits for the same part:

1. **survey** - `TG_P5Survey.mod` drives the robot to every TD05Touch pose and records the joint
   solution it takes there and the station's axes. The planner export needs both: it writes
   `MoveAbsJ` postures, derives confdata from joints and writes the station axes into every
   robtarget. A real project gets them from its IK; this one measures them on the controller.
2. **export** - `planner_touch_export.py` builds the touch programs with the planner's REAL touch
   builders (`WeldTouchGenerationService._build_touch_program` / `_build_wrapper_program`), a
   weld with the ABB weld blocks, a package main with the package blocks, and compiles it with
   the planner's `ProgramCompiler` + `AbbTranslator` under the ABB HMI profile. It also writes the
   touch points into a `.tgs` the way the planner persists them, which the prototype HMI then
   reads the way the real HMI does (`abb_server.load_tgs_touch_points`).
3. **run** - the experiments, each one real TG cycle (`TG_TrRig.TG_TrRun` -> `tgs_main`) with the
   prototype HMI serving the planner's module and the .tgs touch points:
   - E1 static: the module passes `rapid_check`, and its touch block has TD05Touch's shape;
   - E2 baseline / shifted / aligned: as P3, and the weld pass must follow the corrected frame
     (TCP trace during the arc);
   - E3 touch off (id 16 = 0): no search, the weld runs at the nominal frame;
   - E4 malformed TSP frame: the new abort line ends the run before any touch request;
   - E5 miss, pause, Start: the block is moved out of reach, the first search misses and
     pauses (D3); the block is put back and Start searches again - the wire stays in lockstep.

Nothing is written into curobo_suite: it is imported read-only (no bytecode, no caches).
Needs the P3 prerequisites (rig signals, P1/P2 RAPID, PM idle, one station in position).
"""
import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import touch_probe as tp  # noqa: E402
from touch_search_vc import build_ok, module_names  # noqa: E402

OUT = os.path.join(HERE, "planner_out")
SURVEY_MODULE = "TG_P5Survey"
SURVEY_POSES = ("Home", "Via", "A1", "C1", "A2", "C2", "WA", "WS", "WE", "WD")


def log(msg):
    tp.log(msg)


def _sym(module, name):
    body = tp.get("/rw/rapid/symbol/data/RAPID/T_ROB1/%s/%s" % (module, name))
    m = re.search(r'"value":\s*"((?:\\.|[^"\\])*)"', body)
    return m.group(1).replace('\\"', '"') if m else "?"


def _numbers(text):
    return [float(v) for v in re.findall(r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?", text)]


def load_module(name):
    if tp.exec_state() != "stopped":
        tp.stop()
    mark = tp.elog_mark()
    with open(os.path.join(HERE, name + ".mod"), "rb") as fh:
        tp.c.put_file("$home/TGS/%s.mod" % name, fh.read())
    tp.c.request_mastership()
    try:
        if name in module_names():
            tp.post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": name})
            time.sleep(1.0)
        tp.post("/rw/rapid/tasks/T_ROB1", {"action": "loadmod"}, {"modulepath": "$home/TGS/%s.mod" % name})
    finally:
        tp.c.release_mastership()
    time.sleep(2.0)
    for e in tp.elog_after(mark):
        log("  elog [%s] %s | %s" % e[1:])
    return build_ok()


def unload_module(name):
    if tp.exec_state() != "stopped":
        tp.stop()
    tp.c.request_mastership()
    try:
        tp.post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": name})
    finally:
        tp.c.release_mastership()
    log("%s unloaded" % name)


def run_to_stop(module, routine, timeout=180.0):
    if tp.exec_state() != "stopped":
        tp.stop()
    tp.c.request_mastership()
    try:
        tp.post("/rw/rapid/tasks/T_ROB1/pcp", {"action": "set-pp-routine"},
                {"module": module, "routine": routine, "userlevel": "FALSE"})
    finally:
        tp.c.release_mastership()
    time.sleep(0.5)
    t = time.time()
    tp.start("once")
    time.sleep(0.5)
    while tp.exec_state() != "stopped" and time.time() - t < timeout:
        time.sleep(0.3)
    return time.time() - t


def preflight():
    tp.connect()
    log("VC %s, execution %s, PP in %r" % (tp.URL, tp.exec_state(), tp.pcp_module()))
    if tp.exec_state() != "stopped" and tp.pcp_module() != "gapMain":
        log("REFUSED: PP in %r, not PM's idle gapMain - a cycle may be running" % tp.pcp_module())
        return False
    at = [re.search(r'"lvalue":"([^"]*)"', tp.get("/rw/iosystem/signals/Local/B_GAP_SIM/siGap_AtStn_%d" % n)).group(1)
          for n in (1, 2)]
    if sorted(at) != ["0", "1"]:
        log("REFUSED: siGap_AtStn_1/2 = %s - TG_ActMechUnit needs exactly one station in position" % at)
        return False
    return True


def survey():
    """TG_P5Survey -> planner_out/survey.json: {pose: {"robax_deg", "extax"}}, station, tool."""
    if not preflight():
        return 1
    if not load_module(SURVEY_MODULE):
        log("SURVEY MODULE FAILED TO BUILD")
        return 1
    try:
        mark = tp.elog_mark()
        seconds = run_to_stop(SURVEY_MODULE, "TG_P5Run")
        done = _sym(SURVEY_MODULE, "nP5Done")
        poses = {}
        for name in SURVEY_POSES:
            v = _numbers(_sym(SURVEY_MODULE, "jtP5" + name))
            poses[name] = {"robax_deg": v[0:6], "extax": v[6:12]}
        station = 1 if re.search(r'"lvalue":"1"', tp.get(
            "/rw/iosystem/signals/Local/B_GAP_SIM/siGap_AtStn_1")) else 2
        result = {"done": done, "seconds": round(seconds, 1), "station": station,
                  "tool": _sym("TG_Comms", "tTG_Weld"), "poses": poses,
                  "elog": tp.elog_after(mark)}
        os.makedirs(OUT, exist_ok=True)
        with open(os.path.join(OUT, "survey.json"), "w") as fh:
            json.dump(result, fh, indent=1, default=str)
        log("survey: done=%s in %.1f s, station %d, tool %s" % (done, seconds, station, result["tool"]))
        for name in SURVEY_POSES:
            log("  %-4s robax %s extax %s" % (name, ["%.3f" % a for a in poses[name]["robax_deg"]],
                                              poses[name]["extax"]))
        for e in result["elog"]:
            log("  elog [%s] %s | %s" % e[1:])
        return 0 if done.strip() == "1" else 1
    finally:
        unload_module(SURVEY_MODULE)
        tp.back_to_pm()


def export():
    """The planner export (read-only on curobo_suite) -> planner_out/TsPlan05.mod + .tgs."""
    import planner_touch_export as pe
    meta = pe.export(os.path.join(OUT, "survey.json"), OUT)
    log("exported %s (%d warnings) and %s" % (meta["module"], len(meta["warnings"]), meta["tgs"]))
    for warning in meta["warnings"]:
        log("  planner warning: " + warning[:150])
    return 0


# ---------------------------------------------------------------------------------------- run
PLANNER_LOG = ["10", "5", "4", "16", "18", "18", "19", "4", "14", "100"]
TOUCH_OFF_LOG = ["10", "5", "4", "16", "4", "14", "100"]
CORRUPT_TSP_LOG = ["10", "5", "4", "100"]
# The rig's World Zone is sampled every 24 ms (X7), so one quantum at 15 mm/s is 0.36 mm.
QUANTUM = 15.0 * 0.024
# The weld line must sit where the CONTROLLER's corrected frame puts it: the HMI's delta, to
# the TCP-report resolution (RWS reports the TCP to 1e-3 mm; the arc runs fine-point to fine-point).
WELD_LINE_TOL = 0.05
# The prototype HMI's R_W_P travel speed (abb_server.AbbTgsHmi.travel_speed).
WELD_TRAVEL_IPM = 17.5
PAUSE_S = 10.0


class _Recorder:
    def __init__(self):
        self.t0 = time.perf_counter()
        self.events = []

    def now(self):
        return round(time.perf_counter() - self.t0, 3)


def _meta():
    with open(os.path.join(OUT, "TsPlan05_export.json")) as fh:
        return json.load(fh)


def _event_time(events, suffix):
    times = [t for t, m in events if m == "serving request " + suffix]
    return times[0] if times else None


def _weld_line_check(r, meta, delta_cad, rows, label):
    """The arc span, from the TCP trace: the weld runs on the line the corrected frame puts it on.

    Only samples between the weld's R_W_P (id 14, right before ArcLStart) and the end request
    count: touch 2's vertical search crosses the seam's line, and its samples are not the weld."""
    from abb_server import rotate_by_wpr
    t_from, t_to = _event_time(r["events"], "14"), _event_time(r["events"], "100")
    if t_from is None or t_to is None:
        rows.append(("%s: weld pass traced on the corrected seam" % label, False, "no R_W_P / R_E in the log"))
        return None
    samples = [s for s in r["samples"] if t_from <= s[0] <= t_to]
    frame = meta["weld_frame_xyzwpr"]
    (xs, ys, zs), (xe, ye, ze) = meta["weld_line_mm"]
    delta_base = rotate_by_wpr(frame[3], frame[4], frame[5], delta_cad)

    def world(p):
        v = rotate_by_wpr(frame[3], frame[4], frame[5], p)
        return tuple(f + a + d for f, a, d in zip(frame[:3], v, delta_base))

    start, end = world((xs, ys, zs)), world((xe, ye, ze))
    # Samples on the seam: between the two ends along the seam, off the ends by 3 mm.
    axis = [e - s for s, e in zip(start, end)]
    length = sum(a * a for a in axis) ** 0.5
    unit = [a / length for a in axis]
    on_seam = []
    for t, x, y, z in samples:
        rel = [x - start[0], y - start[1], z - start[2]]
        along = sum(r * u for r, u in zip(rel, unit))
        off = [r - along * u for r, u in zip(rel, unit)]
        if 3.0 < along < length - 3.0 and sum(o * o for o in off) ** 0.5 < 5.0:
            on_seam.append((t, off))
    if len(on_seam) < 5:
        rows.append(("%s: weld pass traced on the corrected seam" % label, False,
                     "%d samples near it (world %s -> %s)" % (len(on_seam), start, end)))
        return None
    worst = max(max(abs(v) for v in off) for _, off in on_seam)
    span = on_seam[-1][0] - on_seam[0][0]
    rows.append(("%s: weld pass runs on the corrected seam" % label, worst <= WELD_LINE_TOL,
                 "%d samples over %.1f s, max off-line %.3f mm (need <= %.2f); seam world %s -> %s"
                 % (len(on_seam), span, worst, WELD_LINE_TOL,
                    ["%.2f" % v for v in start], ["%.2f" % v for v in end])))
    # The arc's speed is welddata's (TG_ApplyWeldParams from the HMI's R_W_P), not the move's.
    speed = (length - 6.0) / span if span > 0 else 0.0
    wanted = WELD_TRAVEL_IPM * 25.4 / 60.0
    rows.append(("%s: the arc runs at the HMI's travel speed" % label, abs(speed - wanted) <= 0.02 * wanted,
                 "%.3f mm/s over the seam (the HMI's %.1f IPM = %.3f mm/s, need within 2 %%)"
                 % (speed, WELD_TRAVEL_IPM, wanted)))
    return {"samples": len(on_seam), "seconds": span, "max_off_mm": worst, "speed_mm_s": speed}


def planner_cycle(label, shift, rec, *, auto=True, corrupt_tsp=False, miss_first=False, hmi_base=None,
                  hmi_setup=None):
    """One real TG cycle serving the planner's module; returns the evidence.

    hmi_base: the HMI class (default the prototype, touch_real_vc.LoggingHmi); P4 passes the C++
    harness behind the same interface (p4_cpp_touch_vc.CppHmi). hmi_setup(hmi) sets it up further."""
    import threading
    import touch_real_vc as tr
    from touch_demo_vc import TcpSampler, now as trace_now
    from abb_server import load_tgs_touch_points, rotate_by_wpr
    from touch_search_vc import set_sym, sym
    import touch_probe_math as tpm

    meta = _meta()
    frame = meta["weld_frame_xyzwpr"]
    world_shift = rotate_by_wpr(frame[3], frame[4], frame[5], shift)
    set_sym(tr.RIG, "nTrShiftY", "%.3f" % world_shift[1])
    set_sym(tr.RIG, "nTrShiftZ", "%.3f" % world_shift[2])
    pauses_before = float(sym("TG_Touch", "nTG_TouchPauses"))
    mark = tp.elog_mark()
    events = []

    class EventHmi(hmi_base or tr.LoggingHmi):
        def _log(self, msg):
            events.append((trace_now(), msg))       # the TCP sampler's clock
            super()._log(msg)

    sampler = TcpSampler()
    sampler.start()
    tr.pp_and_start("TG_TrRun")
    t = time.time()
    while tp.exec_state() != "running" and time.time() - t < 5:
        time.sleep(0.2)
    time.sleep(3.5)       # the restarted tgs_main closes the stale handshake listener first (§20)
    if miss_first:
        # E5: the touch DI is pointed at a signal that never rises, so the first search runs its
        # whole stroke and pauses (D3); the rig's signal is put back while it waits.
        set_sym("TG_Cell", "stTG_TouchDI", '"diWld1Touched"')
    hmi = EventHmi(host="127.0.0.1", port=2000, handshake_port=2001, rws=tp.URL, verbose=True)
    hmi.prog_name = meta["prog_name"]
    hmi.tgs_source_dir = OUT
    hmi.weld_frame_xyzwpr = list(frame)
    hmi.touch_points_by_weld = load_tgs_touch_points(meta["tgs"])
    hmi.auto_touchups = bool(auto)
    hmi.corrupt_weld_frame = bool(corrupt_tsp)
    if hmi_setup is not None:
        hmi_setup(hmi)
    error = []

    def serve():
        try:
            hmi.serve_cycle()
        except Exception as exc:
            error.append(exc)

    worker = threading.Thread(target=serve, daemon=True)
    worker.start()
    pause_seen = None
    stopped_since = None
    while worker.is_alive():
        worker.join(0.3)
        if tp.exec_state() == "stopped" and worker.is_alive():
            paused = float(sym("TG_Touch", "nTG_TouchPauses")) - pauses_before
            if miss_first and pause_seen is None and paused >= 1:
                pause_seen = {"t": trace_now(), "reason": float(sym("TG_Touch", "nTG_TouchLastPause")),
                              "reports_before": sum(1 for _, m in events if m.startswith("serving request 18"))}
                events.append((trace_now(), "PAUSED (reason %d) - rig signal back, Start" % pause_seen["reason"]))
                log("  paused, reason %d, %d touch reports so far - putting the rig signal back, Start"
                    % (pause_seen["reason"], pause_seen["reports_before"]))
                set_sym("TG_Cell", "stTG_TouchDI", '"diTG_SimTouched"')
                time.sleep(1.0)
                tp.start("once")
                time.sleep(1.0)
            elif paused >= (2 if miss_first else 1):
                log("  UNEXPECTED PAUSE (reason %s) - ending the cycle" % sym("TG_Touch", "nTG_TouchLastPause"))
                break
            elif pause_seen is None or trace_now() - pause_seen["t"] > 5.0:
                if stopped_since is None:
                    stopped_since = time.time()
                elif time.time() - stopped_since > 5.0:
                    log("  PROGRAM STOPPED with no touch pause - an error? ending the cycle")
                    break
        else:
            stopped_since = None
        if time.time() - t > 240:
            log("  TIMEOUT - the cycle did not finish")
            break
    time.sleep(3.0)
    state = tp.exec_state()
    tp.stop()
    sampler.halt.set()
    sampler.join(2.0)
    ctl = {"oframe": tpm.parse_wobj(sym("TG_Comms", "wobjTG_Weld"))["oframe"],
           "pauses": float(sym("TG_Touch", "nTG_TouchPauses")) - pauses_before,
           "last_pause": float(sym("TG_Touch", "nTG_TouchLastPause")),
           "weld_status": float(sym("TG_Comms", "nTG_WeldStatus")),
           "zone_made": float(sym(tr.RIG, "nTrZoneMade")), "state_after": state}
    ev = tp.elog_after(mark)
    log("=== %s %s: request log %s, pauses %d, weld status %d" % (label, shift, hmi.request_log, ctl["pauses"],
                                                              ctl["weld_status"]))
    for e in ev:
        if not e[1].startswith(("100", "80003")):
            log("  elog [%s] %s | %s" % (e[1], e[2].replace("\\n", " ").strip(), e[3].replace("\\n", " ").strip()[:110]))
    return {"hmi_log": hmi.request_log, "measured": hmi.last_touch_points, "offset": hmi.touch_offset_base,
            "frame19": hmi.last_touch_frame_xyzwpr, "ctl": ctl, "error": repr(error[0]) if error else None,
            "elog": ev, "events": events, "samples": sampler.samples, "pause": pause_seen, "shift": shift,
            "cpp": getattr(hmi, "cpp_result", None)}


def _judge_touch(label, shift, r, rows, expected_log):
    import touch_real_vc as tr
    tr.EXPECTED_LOG = list(expected_log)
    return tr.judge(label, shift, r, rows)


def _search_measurements(r, meta):
    """From the trace: per touch, the search's duration and how far past the hit the TCP stopped.

    The search is what happens after the TCP's LAST arrival at the touch's approach before the
    report: there it starts (TG_TouchSearch's fine move is a zero-length one), and where the TCP
    stands at the report is where SearchL \\Stop left it (no auto return, D12). Measured along the
    search direction, from the NOMINAL approach -- the TSP frame is the uncorrected one."""
    from abb_server import rotate_by_wpr
    frame = meta["weld_frame_xyzwpr"]
    out = []
    samples = r["samples"]
    reports = [t for t, m in r["events"] if m == "serving request 18"]
    for index, (touch, hit) in enumerate(zip(meta["touches"], r["measured"] or [])):
        k = [abs(v) for v in touch["snapped_axis"]].index(1.0)
        wdir = rotate_by_wpr(frame[3], frame[4], frame[5], [-v for v in touch["snapped_axis"]])
        wstart = [f + v for f, v in zip(frame[:3], rotate_by_wpr(frame[3], frame[4], frame[5], touch["approach"]))]
        if index >= len(reports):
            continue
        before = [s for s in samples if s[0] <= reports[index]]
        at_start = [i for i, s in enumerate(before)
                    if sum((p - q) ** 2 for p, q in zip(s[1:], wstart)) ** 0.5 < 0.3]
        if not at_start:
            out.append({"touch": index + 1, "note": "the approach is not in the trace"})
            continue
        search = before[at_start[-1]:]
        progress = [(s[0], sum((p - q) * d for p, q, d in zip(s[1:], wstart, wdir))) for s in search]
        rest = progress[-1][1]
        t_go = next((t for t, p in progress if p > 0.05), progress[0][0])
        t_stop = next((t for t, p in progress if p >= rest - 0.01), progress[-1][0])
        hit_travel = abs(float(hit[k]) - float(touch["approach"][k]))
        out.append({"touch": index + 1, "search_s": round(t_stop - t_go, 2),
                    "travel_to_hit_mm": round(hit_travel, 3), "travel_to_rest_mm": round(rest, 3),
                    "overshoot_mm": round(rest - hit_travel, 3),
                    "mean_search_speed_mm_s": round(rest / (t_stop - t_go), 2) if t_stop > t_go else None})
    return out


def run():
    """E1-E5 against the rig; results -> planner_out/planner_run_<time>.json."""
    import touch_real_vc as tr
    from touch_search_vc import sym
    if not preflight():
        return 1
    if "TG_Touch" not in module_names():
        log("REFUSED: TG_Touch is not loaded - run touch_search_vc.py (P2 deploy) first")
        return 1
    meta = _meta()
    rows, results = [], {}
    # E1 static: rapid_check, and the touch block's shape against TD05Touch.
    rows.extend(static_checks(meta))
    if not tr.load_rig():
        log("RIG LOAD FAILED")
        return 1
    rec = _Recorder()
    q = QUANTUM
    plan = [
        ("baseline", (0.0, 0.0, 0.0), {}),
        ("shifted", (3.0, 0.0, -2.0), {}),
        ("aligned", (9 * q, 0.0, -6 * q), {}),
        ("touch-off", (3.0, 0.0, -2.0), {"auto": False}),
        ("corrupt-tsp", (0.0, 0.0, 0.0), {"corrupt_tsp": True}),
        ("miss-retry", (0.0, 0.0, 0.0), {"miss_first": True}),
    ]
    deltas = {}
    try:
        for i, (label, shift, opts) in enumerate(plan):
            if i:
                time.sleep(PAUSE_S)
            r = planner_cycle(label, shift, rec, **opts)
            results[label] = r
            if label in ("baseline", "shifted", "aligned"):
                deltas[label] = _judge_touch(label, shift, r, rows, PLANNER_LOG)
                if deltas[label] is not None:
                    r["weld_line"] = _weld_line_check(r, meta, deltas[label], rows, label)
                    r["searches"] = _search_measurements(r, meta)
            elif label == "touch-off":
                ok = r["error"] is None and r["hmi_log"] == TOUCH_OFF_LOG and r["ctl"]["pauses"] == 0
                rows.append(("touch-off: id 16 = 0 skips the whole block", ok, "%s, pauses %s" % (r["hmi_log"], r["ctl"]["pauses"])))
                trans = r["ctl"]["oframe"][0]
                dmm = max(abs(a - b) for a, b in zip(trans, meta["weld_frame_xyzwpr"][:3]))
                rows.append(("touch-off: the weld frame stays the served nominal", dmm <= 0.006,
                             "oframe %s, |dmm| %.4f" % (["%.3f" % v for v in trans], dmm)))
                r["weld_line"] = _weld_line_check(r, meta, (0.0, 0.0, 0.0), rows, label)
            elif label == "corrupt-tsp":
                ok = r["error"] is None and r["hmi_log"] == CORRUPT_TSP_LOG
                rows.append(("corrupt-tsp: a malformed touch frame ends the run before any touch request", ok,
                             "%s, weld status %s" % (r["hmi_log"], r["ctl"]["weld_status"])))
                moved = [s for s in r["samples"] if abs(s[3] - r["samples"][0][3]) > 5.0]
                rows.append(("corrupt-tsp: the robot never left home", not moved,
                             "%d samples more than 5 mm from the start height" % len(moved)))
            elif label == "miss-retry":
                p = r["pause"] or {}
                rows.append(("miss-retry: the first search missed and paused (D3, reason 3)",
                             p.get("reason") == 3.0 and p.get("reports_before") == 0,
                             "pause %s" % p))
                deltas[label] = _judge_touch(label, shift, dict(r, ctl=dict(r["ctl"], pauses=0)), rows, PLANNER_LOG)
                if deltas[label] is not None:
                    r["weld_line"] = _weld_line_check(r, meta, deltas[label], rows, label)
                rows.append(("miss-retry: exactly one pause, the wire stayed in lockstep",
                             r["ctl"]["pauses"] == 1 and r["hmi_log"] == PLANNER_LOG,
                             "pauses %s, log %s" % (r["ctl"]["pauses"], r["hmi_log"])))
        if deltas.get("baseline") and deltas.get("aligned"):
            diff = [a - b for a, b in zip(deltas["aligned"], deltas["baseline"])]
            err = max(abs(a - b) for a, b in zip(diff, plan[2][1]))
            rows.append(("aligned - baseline = the aligned shift", err <= 0.03,
                         "%s, max err %.4f (need <= 0.030)" % (["%.3f" % v for v in diff], err)))
        if deltas.get("baseline") and deltas.get("miss-retry"):
            err = max(abs(a - b) for a, b in zip(deltas["miss-retry"], deltas["baseline"]))
            rows.append(("miss-retry measures what baseline measured", err <= 0.03,
                         "max diff %.4f mm (need <= 0.030)" % err))
        p3 = p3_reference()
        for label in ("baseline", "aligned"):
            if p3 and deltas.get(label) and p3.get(label):
                err = max(abs(a - b) for a, b in zip(deltas[label], p3[label]))
                rows.append(("%s: the planner's module measures what TD05Touch measured (P3)" % label, err <= 0.03,
                             "planner %s vs TD05Touch %s, max diff %.4f"
                             % (["%.3f" % v for v in deltas[label]], ["%.3f" % v for v in p3[label]], err)))
    finally:
        out = os.path.join(OUT, time.strftime("planner_run_%Y%m%d_%H%M%S.json"))
        with open(out, "w") as fh:
            json.dump({"results": results, "rows": rows}, fh, indent=1, default=str)
        log("raw results: %s" % out)
        for step in (lambda: tr.run_quick("TG_TrRestore"), lambda: tr.run_quick("TG_TrHome"), tr.unload_rig,
                     tp.back_to_pm):
            try:
                step()
            except Exception as exc:
                log("CLEAN-UP STEP FAILED: %s - finish by hand (TG_TrRestore, park, unload TG_TrRig, PP to main)" % exc)
        names = {k: sym("TG_Cell", k) for k in ("stTG_TouchDI", "stTG_TouchOnDO", "stTG_TouchActiveDI")}
        rows.append(("TG_Cell welder names restored", names == {"stTG_TouchDI": '"diWld1Touched"',
                     "stTG_TouchOnDO": '"doWld1TouchOn"', "stTG_TouchActiveDI": '"diWld1TouchActive"'}, "%s" % names))
    log("")
    for label in ("baseline", "shifted", "aligned"):
        for m in (results.get(label) or {}).get("searches") or []:
            log("measure %-8s touch %s" % (label, m))
        wl = (results.get(label) or {}).get("weld_line")
        if wl:
            log("measure %-8s weld pass %s" % (label, wl))
    for name, ok, detail in rows:
        log("%s  %s -- %s" % ("PASS" if ok else "FAIL", name, detail))
    return 0 if rows and all(ok for _, ok, _ in rows) else 1


def p3_reference():
    """P3's TD05Touch deltas (touch_real_20260926_095252.json), the hand-written spec's numbers."""
    path = os.path.join(HERE, "touch_real_20260926_095252.json")
    if not os.path.exists(path):
        return None
    with open(path) as fh:
        data = json.load(fh)
    import touch_real_vc as tr
    out = {}
    for label in ("baseline", "aligned"):
        r = data["results"].get(label)
        if r and r.get("measured"):
            out[label] = tr.delta_cad(r["measured"])
    return out


def _statements(text, start_marker, end_marker):
    lines = [ln.strip() for ln in text.splitlines()]
    body = []
    inside = False
    for ln in lines:
        if not inside and re.match(start_marker, ln):
            inside = True
        if inside:
            if ln and not ln.startswith("!"):
                body.append(ln)
            if re.match(end_marker, ln):
                break
    return body


def _shape(statements):
    """Statement kinds, with targets/speeds/conf toggles and entry guards folded away."""
    out, skip = [], False
    for s in statements:
        if s.startswith("IF Distance(CPos("):
            skip = True
            continue
        if skip:
            skip = s != "ENDIF"
            continue
        if s.startswith(("ConfL", "ConfJ")):
            continue
        head = re.split(r"[ ;(\\]", s, 1)[0]
        if head in ("MoveJ", "MoveL", "MoveAbsJ"):
            head = "Move"
        if out and out[-1] == head == "Move":
            continue
        out.append(head if head not in ("IF",) else s.split(" THEN")[0].split(" GOTO")[0])
    return out


def static_checks(meta):
    """E1: the module lints clean, and its touch block has TD05Touch's shape."""
    import subprocess
    rows = []
    root = os.path.normpath(os.path.join(HERE, "..", ".."))
    rapid = os.path.join(root, "abb", "rapid")
    lint = subprocess.run([sys.executable, os.path.join(root, "tools", "rapid_check.py"), meta["module"]]
                          + [os.path.join(rapid, f) for f in ("TG_Comms.sys", "TG_Cell.sys", "TG_Touch.sys", "TG_Weld.sys")],
                          capture_output=True, text=True)
    rows.append(("E1: the planner's module passes rapid_check with the resident modules", lint.returncode == 0,
                 (lint.stdout.strip().splitlines() or ["?"])[-1]))
    with open(meta["module"]) as fh:
        planner = fh.read()
    with open(os.path.join(rapid, "TGS", "TD05Touch.mod")) as fh:
        spec = fh.read()
    a = _shape(_statements(planner, r'stTG_SubName:="TSP', r"TG_ReqTouchSenseEnd"))
    b = _shape(_statements(spec, r'stTG_SubName:="TSP', r"TG_ReqTouchSenseEnd"))
    rows.append(("E1: the touch block has TD05Touch's statement shape", a == b,
                 "planner %s | TD05Touch %s" % (" ".join(a), " ".join(b))))
    rows.append(("E1: the planner emits no FANUC touch syntax", not re.search(r"R_TS_|Search Start|Search\[|CALL ", planner),
                 "grep R_TS_ / Search Start / Search[ / CALL"))
    return rows


def main(argv):
    what = argv[0] if argv else "all"
    steps = {"survey": survey, "export": export, "run": run}
    if what == "all":
        for name in ("survey", "export", "run"):
            code = steps[name]()
            if code:
                return code
            if name != "run":
                time.sleep(PAUSE_S)
        return 0
    if what in steps:
        return steps[what]()
    raise SystemExit("usage: planner_touch_vc.py survey|export|run|all")


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
