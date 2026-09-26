"""Touch-sense P3 on the MONARC VC: TGS/TD05Touch.mod, real searches, end to end.

    python touch_real_vc.py

docs/abb_touch_sense_port_v1.md §6 P3; recipe and expected output in robotstudio_setup.md §22.

**Two cycles**, each running the real TG cycle (``TG_TrRig.TG_TrRun`` → ``tgs_main``) against
the prototype HMI in ``touch-real`` mode:
- **baseline**: the rig's block exactly where TD05Touch's nominal points put it;
- **shifted**: the block moved +3.000 mm in part X (world +Y) and -2.000 mm in part Z.

**What is judged:**
- the HMI's per-touch measurements against the block faces;
- the HMI's offset against the shift;
- shifted minus baseline, which cancels the World Zone trigger lead;
- the controller's own ``wobjTG_Weld.oframe`` against the frame the HMI meant.

**Needs:**
- the World Zone rig signals (``touch_probe.py setup-io``);
- the P1 ``TG_Comms.sys`` and the P2 ``TG_Cell.sys`` + ``TG_Touch.sys`` loaded;
- PM idle, with exactly one station in position.

At the end it runs ``TG_TrRestore``, parks the robot, unloads the rig and restarts PM.
"""
import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import touch_probe as tp  # noqa: E402
import touch_probe_math as tpm  # noqa: E402
from abb_server import (TOUCH_REAL_DEMO, AbbTgsHmi, euler_wpr_to_quat, rotate_by_wpr,  # noqa: E402
                        touch_axis_index, translate_frame)
from touch_search_vc import build_ok, module_names, set_sym, sym  # noqa: E402

RIG = "TG_TrRig"
PAUSE_S = 10.0
# Part-frame shifts, mm. The rig sees a touch only every 24 ms = 0.36 mm at 15 mm/s (X7,
# touch_probe_math.RIG_PERIOD_S), always up to one quantum early:
#   baseline  - the block where the nominals put it;
#   shifted   - the plan's +3 / -2: an arbitrary shift, so each touch may land anywhere in the
#               quantum - judged against the rig's resolution;
#   aligned   - +3.24 / -2.16 = 9 and 6 quanta: every touch crosses its face at the SAME phase
#               as in the baseline, so aligned - baseline must reproduce the shift almost exactly.
QUANTUM = tpm.SEARCH_SPEED * tpm.RIG_PERIOD_S
SHIFTS = (("baseline", (0.0, 0.0, 0.0)), ("shifted", (3.0, 0.0, -2.0)),
          ("aligned", (9 * QUANTUM, 0.0, -6 * QUANTUM)))
SHIFT_TOL = QUANTUM + 0.03      # absolute, and shifted - baseline: one quantum of rig resolution
ALIGNED_TOL = 0.03              # aligned - baseline: the quantization cancels
FRAME_MM = 0.006
QUAT_TOL = 2e-6
EXPECTED_LOG = ["10", "5", "4", "16", "18", "18", "19", "4", "100"]


def log(msg):
    tp.log(msg)


class LoggingHmi(AbbTgsHmi):
    def _log(self, msg):
        if "robot ->" not in msg and "hmi   ->" not in msg and "pose(xyzwpr)" not in msg:
            print("      hmi | " + msg, flush=True)


def load_rig():
    if tp.exec_state() != "stopped":
        tp.stop()
    mark = tp.elog_mark()
    with open(os.path.join(HERE, RIG + ".mod"), "rb") as fh:
        tp.c.put_file("$home/TGS/%s.mod" % RIG, fh.read())
    tp.c.request_mastership()
    try:
        if RIG in module_names():
            tp.post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": RIG})
            time.sleep(1.0)
        tp.post("/rw/rapid/tasks/T_ROB1", {"action": "loadmod"}, {"modulepath": "$home/TGS/%s.mod" % RIG})
    finally:
        tp.c.release_mastership()
    time.sleep(2.0)
    for e in tp.elog_after(mark):
        log("  elog [%s] %s | %s" % e[1:])
    return build_ok()


def unload_rig():
    if tp.exec_state() != "stopped":
        tp.stop()
    tp.c.request_mastership()
    try:
        tp.post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": RIG})
    finally:
        tp.c.release_mastership()
    log("rig module unloaded")


def pp_and_start(routine):
    if tp.exec_state() != "stopped":
        tp.stop()
    tp.c.request_mastership()
    try:
        tp.post("/rw/rapid/tasks/T_ROB1/pcp", {"action": "set-pp-routine"},
                {"module": RIG, "routine": routine, "userlevel": "FALSE"})
    finally:
        tp.c.release_mastership()
    time.sleep(0.5)
    tp.start("once")


def run_quick(routine, timeout=30):
    pp_and_start(routine)
    t = time.time()
    time.sleep(0.5)
    while tp.exec_state() != "stopped" and time.time() - t < timeout:
        time.sleep(0.2)


def run_cycle(label, shift):
    """One TG cycle with the block moved by `shift` (part frame); returns the evidence."""
    frame = TOUCH_REAL_DEMO["weld_frame_xyzwpr"]
    world_shift = rotate_by_wpr(frame[3], frame[4], frame[5], shift)
    set_sym(RIG, "nTrShiftY", "%.3f" % world_shift[1])
    set_sym(RIG, "nTrShiftZ", "%.3f" % world_shift[2])
    pauses_before = sym("TG_Touch", "nTG_TouchPauses")
    mark = tp.elog_mark()
    pp_and_start("TG_TrRun")
    t = time.time()
    while tp.exec_state() != "running" and time.time() - t < 5:
        time.sleep(0.2)
    time.sleep(3.5)       # the restarted tgs_main closes the stale handshake listener first (§20)
    hmi = LoggingHmi(host="127.0.0.1", port=2000, handshake_port=2001, rws=tp.URL, verbose=True)
    hmi.prog_name = TOUCH_REAL_DEMO["prog_name"]
    hmi.weld_frame_xyzwpr = list(frame)
    hmi.touch_points = [dict(p) for p in TOUCH_REAL_DEMO["touch_points"]]
    hmi.auto_touchups = True
    err = None
    try:
        hmi.serve_cycle()
    except Exception as exc:
        err = exc
        log("  HMI raised: %r" % exc)
    time.sleep(3.0)
    state = tp.exec_state()
    tp.stop()
    ctl = {"oframe": tpm.parse_wobj(sym("TG_Comms", "wobjTG_Weld"))["oframe"],
           "pauses": float(sym("TG_Touch", "nTG_TouchPauses")) - float(pauses_before),
           "zone_made": float(sym(RIG, "nTrZoneMade")), "state_after": state}
    ev = tp.elog_after(mark)
    log("=== %s %s: request log %s, pauses %d, zone builds %d" % (label, shift, hmi.request_log, ctl["pauses"],
                                                                 ctl["zone_made"]))
    for e in ev:
        if e[1].startswith("800") or e[1] in ("40322", "40160", "40223", "41617", "40574", "40661"):
            log("  elog [%s] %s | %s" % (e[1], e[2].replace("\\n", " ").strip(), e[3].replace("\\n", " ").strip()[:110]))
    return {"hmi_log": hmi.request_log, "measured": hmi.last_touch_points, "offset": hmi.touch_offset_base,
            "frame19": hmi.last_touch_frame_xyzwpr, "ctl": ctl, "error": repr(err) if err else None,
            "elog": ev}


def delta_cad(measured):
    """Per touched axis: measured - nominal, in the part frame (the HMI's own reading)."""
    d = [0.0, 0.0, 0.0]
    for point, xyz in zip(TOUCH_REAL_DEMO["touch_points"], measured):
        k = touch_axis_index(point["snapped_axis"])
        d[k] = xyz[k] - point["nominal"][k]
    return d


def judge(label, shift, r, rows):
    ok_run = r["error"] is None and r["hmi_log"] == EXPECTED_LOG and r["ctl"]["pauses"] == 0
    rows.append(("%s: the full touch block ran, no pause" % label, ok_run,
                 "%s, pauses %s %s" % (r["hmi_log"], r["ctl"]["pauses"], r["error"] or "")))
    if not r["measured"]:
        rows.append(("%s: measurements present" % label, False, "none"))
        return None
    d = delta_cad(r["measured"])
    for point, xyz in zip(TOUCH_REAL_DEMO["touch_points"], r["measured"]):
        k = touch_axis_index(point["snapped_axis"])
        direction = -point["snapped_axis"][k]                 # the search direction on that axis
        face = point["nominal"][k] + shift[k]
        past = (xyz[k] - face) * direction                    # how far past the face the hit sits
        lo, hi = tpm.hit_window(tpm.SEARCH_SPEED)
        rows.append(("%s: touch on axis %s at the (moved) face" % (label, "XYZ"[k]), lo <= past <= hi,
                     "%.3f mm past the face (need %.3f..%.3f); measured %s" % (past, lo, hi, tpm._fmt(xyz))))
    err = max(abs(a - b) for a, b in zip(d, shift))
    rows.append(("%s: HMI offset = the block shift" % label, err <= SHIFT_TOL,
                 "delta_cad %s vs %s (max err %.3f, need <= %.2f)" % (tpm._fmt(d), tpm._fmt(shift), err, SHIFT_TOL)))
    frame = TOUCH_REAL_DEMO["weld_frame_xyzwpr"]
    want = translate_frame(frame, r["offset"]) if r["offset"] is not None else frame
    trans, quat = r["ctl"]["oframe"]
    wq = euler_wpr_to_quat(*want[3:6])
    if sum(a * b for a, b in zip(quat, wq)) < 0:
        wq = tuple(-v for v in wq)
    dmm = max(abs(a - b) for a, b in zip(trans, want[:3]))
    dq = max(abs(a - b) for a, b in zip(quat, wq))
    rows.append(("%s: controller oframe = the frame the HMI served" % label, dmm <= FRAME_MM and dq <= QUAT_TOL,
                 "oframe %s, |dmm| %.4f, |dq| %.1e" % (tpm._fmt(trans), dmm, dq)))
    return d


def main(argv):
    tp.connect()
    log("VC %s, execution %s, PP in %r" % (tp.URL, tp.exec_state(), tp.pcp_module()))
    if tp.exec_state() != "stopped" and tp.pcp_module() != "gapMain":
        log("REFUSED: PP in %r, not PM's idle gapMain - a cycle may be running" % tp.pcp_module())
        return 1
    at = [re.search(r'"lvalue":"([^"]*)"', tp.get("/rw/iosystem/signals/Local/B_GAP_SIM/siGap_AtStn_%d" % n)).group(1)
          for n in (1, 2)]
    if sorted(at) != ["0", "1"]:
        log("REFUSED: siGap_AtStn_1/2 = %s - TG_ActMechUnit needs exactly one station in position" % at)
        return 1
    if "TG_Touch" not in module_names():
        log("REFUSED: TG_Touch is not loaded - run touch_search_vc.py (P2 deploy) first")
        return 1
    if not load_rig():
        log("RIG LOAD FAILED")
        return 1
    rows, results, deltas = [], {}, {}
    try:
        for i, (label, shift) in enumerate(SHIFTS):
            if i:
                time.sleep(PAUSE_S)
            results[label] = run_cycle(label, shift)
            deltas[label] = judge(label, shift, results[label], rows)
        if all(deltas.get(k) is not None for k, _ in SHIFTS):
            for (label, shift), tol in ((SHIFTS[1], SHIFT_TOL), (SHIFTS[2], ALIGNED_TOL)):
                diff = [a - b for a, b in zip(deltas[label], deltas["baseline"])]
                err = max(abs(a - b) for a, b in zip(diff, shift))
                rows.append(("%s - baseline = the shift %s" % (label, tpm._fmt(shift)), err <= tol,
                             "%s, max err %.4f (need <= %.3f)" % (tpm._fmt(diff), err, tol)))
    finally:
        out = os.path.join(HERE, time.strftime("touch_real_%Y%m%d_%H%M%S.json"))
        with open(out, "w") as fh:
            json.dump({"results": results, "rows": rows}, fh, indent=1, default=str)
        log("raw results: %s" % out)
        for step in (lambda: run_quick("TG_TrRestore"), lambda: run_quick("TG_TrHome"), unload_rig,
                     tp.back_to_pm):
            try:
                step()
            except Exception as exc:
                log("CLEAN-UP STEP FAILED: %s - finish by hand (TG_TrRestore, park, unload TG_TrRig, PP to main)" % exc)
        names = {k: sym("TG_Cell", k) for k in ("stTG_TouchDI", "stTG_TouchOnDO", "stTG_TouchActiveDI")}
        rows.append(("TG_Cell welder names restored", names == {"stTG_TouchDI": '"diWld1Touched"',
                     "stTG_TouchOnDO": '"doWld1TouchOn"', "stTG_TouchActiveDI": '"diWld1TouchActive"'}, "%s" % names))
    log("")
    for name, ok, detail in rows:
        log("%s  %s -- %s" % ("PASS" if ok else "FAIL", name, detail))
    return 0 if rows and all(ok for _, ok, _ in rows) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
