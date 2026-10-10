"""Touch-sense P1 on the MONARC VC: the auto touch-up wire (ids 16/18/19), no motion.

    python touch_wire_vc.py            deploy TG_Comms.sys, run the three scenarios, judge
    python touch_wire_vc.py --no-deploy   the same, against the TG_Comms.sys already loaded

docs/abb_touch_sense_port_v1.md §6 P1; recipe and expected output in robotstudio_setup.md §20.

**What it does:**
1. Deploys ``abb/rapid/TG_Comms.sys``: upload, unload, load, then a build check. It rolls
   back to the git HEAD version if the controller reports errors.
2. Runs ``TG_Main.tgs_main`` three times, from PP-to-routine. That is the standalone loop,
   i.e. the whole tgMainCycle: handshake, file transfer, Load, late-bound call.
3. Serves each run in-process with the prototype HMI, which sends ``TGS/TD05TsWire.mod`` over
   RWS at id 10, and gives ``TD05TsWire`` one of three modes:
   - ``touch-wire``: auto touch-ups on;
   - ``touch-off``: id 16 answered 0;
   - ``touch-corrupt``: a malformed id-19 frame.
4. Judges each scenario on the controller's own ``wobjTG_Weld``, read over RWS.
5. Stops RAPID, resets the PP and restarts Production Manager.

It refuses to start unless PM is idle (PP in ``gapMain``) and exactly one station reads in
position (``TG_ActMechUnit`` EXITs otherwise).
"""
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import touch_probe as tp  # noqa: E402  (RWS plumbing: connect/get/post/elog/stop/start/back_to_pm)
import touch_probe_math as tpm  # noqa: E402
from abb_server import (AbbTgsHmi, TOUCH_WIRE_DEMO, euler_wpr_to_quat, rotate_by_wpr,  # noqa: E402
                        translate_frame)

REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
COMMS = os.path.join(REPO, "abb", "rapid", "TG_Comms.sys")
FRAME_MM = 0.006        # the pose literal carries 2 decimals
QUAT_TOL = 2e-6         # ... and 6 decimals, re-normalized by NOrient
# Between scenarios. VC-observed 2026-09-26: after bursts of TG TPWrite lines (~25 per cycle,
# several cycles back to back) the controller logged 41617 "Too intense frequency of Write
# Instructions" and a later TPWrite then blocked for > 75 s, which only a warm restart cleared.
PAUSE_S = 10.0


def log(msg):
    tp.log(msg)


def sym(module, name):
    body = tp.get("/rw/rapid/symbol/data/RAPID/T_ROB1/%s/%s" % (module, name))
    m = re.search(r'"value":\s*"((?:\\.|[^"\\])*)"', body)
    return m.group(1).replace('\\"', '"') if m else "?"


def build_ok():
    tp.c.request_mastership()
    try:
        tp.post("/rw/rapid/tasks/T_ROB1", {"action": "build"}, {})
        return True
    except Exception:
        return False
    finally:
        tp.c.release_mastership()


def load_comms(data):
    mark = tp.elog_mark()
    if tp.exec_state() != "stopped":
        tp.stop()
    tp.c.put_file("$home/TGS/TG_Comms.sys", data)
    tp.c.request_mastership()
    try:
        try:
            tp.post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": "TG_Comms"})
        except Exception as exc:
            log("  (unload TG_Comms: %s)" % str(exc)[-50:])
        time.sleep(1.0)
        tp.post("/rw/rapid/tasks/T_ROB1", {"action": "loadmod"}, {"modulepath": "$home/TGS/TG_Comms.sys"})
    finally:
        tp.c.release_mastership()
    time.sleep(2.0)
    events = tp.elog_after(mark)
    for e in events:
        log("  elog [%s] %s | %s" % e[1:])
    return not any(e[1] == "40322" for e in events) and build_ok()


def deploy():
    with open(COMMS, "rb") as fh:
        new = fh.read()
    if load_comms(new):
        log("TG_Comms.sys deployed, program builds clean")
        return True
    log("DEPLOY FAILED - rolling back to the git HEAD TG_Comms.sys")
    head = subprocess.run(["git", "-C", REPO, "show", "HEAD:abb/rapid/TG_Comms.sys"],
                          capture_output=True, check=True).stdout
    log("rollback %s" % ("OK" if load_comms(head.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")) else "ALSO FAILED"))
    return False


class LoggingHmi(AbbTgsHmi):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.lines = []

    def _log(self, msg):
        self.lines.append(msg)
        print("      hmi | " + msg, flush=True)


def run_scenario(mode):
    """One tgMainCycle served by the prototype HMI in `mode`; returns (hmi, controller data)."""
    if tp.exec_state() != "stopped":
        tp.stop()
    mark = tp.elog_mark()
    tp.c.request_mastership()
    try:
        tp.post("/rw/rapid/tasks/T_ROB1/pcp", {"action": "set-pp-routine"},
                {"module": "TG_Main", "routine": "tgs_main", "userlevel": "FALSE"})
    finally:
        tp.c.release_mastership()
    time.sleep(0.5)
    tp.start("once")
    # VC-observed 2026-09-26: the previous scenario was stopped INSIDE TG_HandshakeCom, and
    # a stopped program keeps its listener. An HMI that connects at once lands on that stale
    # listener (TCP accepts it, nothing ever answers) and times out. The restarted tgs_main
    # closes it in TG_SocketDisc (2 s wait) before opening a fresh one, so wait that out.
    t = time.time()
    while tp.exec_state() != "running" and time.time() - t < 5:
        time.sleep(0.2)
    time.sleep(3.5)
    hmi = LoggingHmi(host="127.0.0.1", port=2000, handshake_port=2001, rws=tp.URL, verbose=True)
    hmi.prog_name = "TD05TsWire"
    hmi.touch_points = [dict(p) for p in TOUCH_WIRE_DEMO["touch_points"]]
    hmi.auto_touchups = mode != "touch-off"
    hmi.corrupt_touch_frame = mode == "touch-corrupt"
    error = None
    try:
        hmi.serve_cycle()
    except Exception as exc:        # a scenario failure is data; the clean-up must still run
        error = exc
        log("  HMI raised: %r" % exc)
    time.sleep(3.0)                 # TG_SocketDisc's 2 s, then the next handshake accept
    tp.stop()
    ctl = {k: sym("TG_Comms", k) for k in ("wobjTG_Weld", "nTG_DoTouchSense", "bTG_TouchHitOK", "rtTG_TouchHit")}
    ev = tp.elog_after(mark)
    log("=== %s: request log %s" % (mode, hmi.request_log))
    for e in ev:
        if e[1].startswith("800") or e[1] in ("40322", "40160", "40223", "41617"):
            log("  elog [%s] %s | %s" % e[1:])
    return hmi, ctl, ev, error


def frame_rows(name, got_oframe, want_xyzwpr):
    trans, quat = got_oframe
    want_q = euler_wpr_to_quat(*want_xyzwpr[3:6])
    if sum(a * b for a, b in zip(quat, want_q)) < 0:     # q and -q are the same rotation
        want_q = tuple(-v for v in want_q)
    d_mm = max(abs(a - b) for a, b in zip(trans, want_xyzwpr[:3]))
    d_q = max(abs(a - b) for a, b in zip(quat, want_q))
    return [(name + ": oframe position", d_mm <= FRAME_MM, "max |diff| %.4f mm, oframe %s" % (d_mm, tpm._fmt(trans))),
            (name + ": oframe rotation unchanged by the touch-up", d_q <= QUAT_TOL, "max |dq| %.2e" % d_q)]


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
    rows = []
    try:
        if "--no-deploy" not in argv and not deploy():
            return 1
        loc = AbbTgsHmi().weld_frame_xyzwpr
        corrected = translate_frame(loc, rotate_by_wpr(loc[3], loc[4], loc[5], TOUCH_WIRE_DEMO["delta_cad"]))

        hmi, ctl, ev, err = run_scenario("touch-wire")
        rows.append(("touch-wire: request order", err is None and hmi.request_log ==
                     ["10", "5", "4", "16", "18", "18", "19", "4", "100"], "%s %s" % (hmi.request_log, err or "")))
        rows.append(("touch-wire: HMI measured the pretended points", [tuple(round(v, 2) for v in p) for p in
                     (hmi.last_touch_points or [])] == [tuple(p) for p in TOUCH_WIRE_DEMO["measured"]],
                     "%s" % hmi.last_touch_points))
        rows.append(("touch-wire: nTG_DoTouchSense = 1, bTG_TouchHitOK consumed",
                     ctl["nTG_DoTouchSense"] == "1" and ctl["bTG_TouchHitOK"] == "FALSE",
                     "%s / %s" % (ctl["nTG_DoTouchSense"], ctl["bTG_TouchHitOK"])))
        rows += frame_rows("touch-wire", tpm.parse_wobj(ctl["wobjTG_Weld"])["oframe"], corrected)

        time.sleep(PAUSE_S)
        hmi, ctl, ev, err = run_scenario("touch-off")
        rows.append(("touch-off: the block is skipped", err is None and hmi.request_log ==
                     ["10", "5", "4", "16", "4", "100"], "%s %s" % (hmi.request_log, err or "")))
        rows.append(("touch-off: nTG_DoTouchSense = 0", ctl["nTG_DoTouchSense"] == "0", ctl["nTG_DoTouchSense"]))
        rows += frame_rows("touch-off", tpm.parse_wobj(ctl["wobjTG_Weld"])["oframe"], loc)

        time.sleep(PAUSE_S)
        hmi, ctl, ev, err = run_scenario("touch-corrupt")
        aborted = [e for e in ev if "touch sensing aborted" in e[2].lower()]
        rows.append(("touch-corrupt: the cycle ends at id 19 (no weld frame, no end request)",
                     hmi.request_log[-1:] == ["19"] and "100" not in hmi.request_log, "%s" % hmi.request_log))
        rows.append(("touch-corrupt: ErrWrite 'TG: touch sensing aborted' in the event log", bool(aborted),
                     "%s" % (aborted[:1] or "none")))
        rows += frame_rows("touch-corrupt (still the TSP frame)", tpm.parse_wobj(ctl["wobjTG_Weld"])["oframe"], loc)
    finally:
        try:
            if tp.exec_state() != "stopped":
                tp.stop()
            tp.back_to_pm()
        except Exception as exc:
            log("CLEAN-UP FAILED: %s - PP to main and start PM by hand" % exc)
    log("")
    for name, ok, detail in rows:
        log("%s  %s -- %s" % ("PASS" if ok else "FAIL", name, detail))
    return 0 if rows and all(ok for _, ok, _ in rows) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
