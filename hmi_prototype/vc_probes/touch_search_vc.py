"""Touch-sense P2 on the MONARC VC: the production search primitive TG_TouchSearch.

    python touch_search_vc.py             deploy TG_Cell.sys + TG_Touch.sys, run S1-S5, judge
    python touch_search_vc.py --no-deploy the same, against the modules already loaded

docs/abb_touch_sense_port_v1.md §6 P2; recipe and expected output in robotstudio_setup.md §21.
It needs the World Zone rig signals (``touch_probe.py setup-io``) and the P1 ``TG_Comms.sys``.

**Deploy:**
1. Backs up the controller's ``HOME:/TGS/TG_Cell.sys``.
2. Swaps in the repo's ``TG_Cell.sys`` and adds ``TG_Touch.sys``.
3. Checks that the program builds; on errors it restores the backup and unloads
   ``TG_Touch``.

**Run:**
- Loads ``TG_TsProbe.mod`` and runs the five scenarios.
- Plays the operator in the paused ones: it waits until ``TG_TouchSearch`` has paused for
  the expected reason (``nTG_TouchLastPause``), applies the fix by writing ONE PERS over RWS,
  and presses Start.
- Finally it calls ``TG_TspRestore`` (TG_Cell's welder names back), parks the robot,
  unloads the probe and restarts Production Manager.
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

REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
RAPID = os.path.join(REPO, "abb", "rapid")
PROBE = "TG_TsProbe"
PAUSE_S = 10.0          # between scenarios: the 41617 lesson (touch_wire_vc.py)
RESULTS = ("stTspStep", "nTspFaceZ", "nTspPauses", "nTspLastPause", "nTspSensorAfter", "bTspLiveAfter",
           "bTspHitOK", "pTspStartPart", "pTspContactPart", "pTspHit", "pTspAfterPart", "pTspAfterW",
           "wobjTspPart")

# name, routine, expected pauses, expected reason, the operator's fix: (module, PERS, value)
SCENARIOS = (
    ("S1 hit", "TG_TspHit", 0, 0, None),
    ("S2 miss, part placed, retry", "TG_TspMissRetry", 1, 3, (PROBE, "nTspZoneWant", "1")),
    ("S3 welder not live, fixed", "TG_TspNotLive", 1, 2, ("TG_Cell", "stTG_TouchActiveDI", '"diTG_SimSensorActive"')),
    ("S4 signal not configured, fixed", "TG_TspNoSignal", 1, 1, ("TG_Cell", "stTG_TouchDI", '"diTG_SimTouched"')),
    ("S5 touching at the start, cleared", "TG_TspAtStart", 1, 4, (PROBE, "nTspZoneWant", "1")),
)


def log(msg):
    tp.log(msg)


def sym(module, name):
    body = tp.get("/rw/rapid/symbol/data/RAPID/T_ROB1/%s/%s" % (module, name))
    m = re.search(r'"value":\s*"((?:\\.|[^"\\])*)"', body)
    return m.group(1).replace('\\"', '"') if m else "?"


def set_sym(module, name, value):
    tp.post("/rw/rapid/symbol/data/RAPID/T_ROB1/%s/%s" % (module, name), {"action": "set"}, {"value": value})


def build_ok():
    tp.c.request_mastership()
    try:
        tp.post("/rw/rapid/tasks/T_ROB1", {"action": "build"}, {})
        return True
    except Exception:
        return False
    finally:
        tp.c.release_mastership()


def module_names():
    return re.findall(r'"name":"([^"]*)"', tp.get("/rw/rapid/modules", {"task": "T_ROB1"}))


def load_modules(files):
    """files: [(module name, bytes)] - unload each (if loaded), upload, load; True if it builds."""
    mark = tp.elog_mark()
    if tp.exec_state() != "stopped":
        tp.stop()
    loaded = module_names()
    tp.c.request_mastership()
    try:
        for name, _ in reversed(files):
            if name in loaded:
                tp.post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": name})
        time.sleep(1.0)
        for name, data in files:
            ext = ".sys"
            tp.c.put_file("$home/TGS/%s%s" % (name, ext), data)
            tp.post("/rw/rapid/tasks/T_ROB1", {"action": "loadmod"}, {"modulepath": "$home/TGS/%s%s" % (name, ext)})
    finally:
        tp.c.release_mastership()
    time.sleep(2.0)
    for e in tp.elog_after(mark):
        if e[1] not in ("10052", "10053"):
            log("  elog [%s] %s | %s" % e[1:])
    return build_ok()


def deploy():
    try:
        backup = tp.c.get_file("$home/TGS/TG_Cell.sys")
    except Exception as exc:
        log("DEPLOY REFUSED: cannot back up the controller's HOME:/TGS/TG_Cell.sys (%s)" % exc)
        return False
    with open(os.path.join(HERE, "TG_Cell_backup_%s.sys" % time.strftime("%Y%m%d_%H%M%S")), "wb") as fh:
        fh.write(backup)
    files = []
    for name in ("TG_Cell", "TG_Touch"):
        with open(os.path.join(RAPID, name + ".sys"), "rb") as fh:
            files.append((name, fh.read()))
    if load_modules(files):
        log("TG_Cell.sys + TG_Touch.sys deployed, program builds clean")
        return True
    log("DEPLOY FAILED - restoring the backed-up TG_Cell.sys, unloading TG_Touch")
    tp.c.request_mastership()
    try:
        if "TG_Touch" in module_names():
            tp.post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": "TG_Touch"})
    finally:
        tp.c.release_mastership()
    log("restore %s" % ("OK" if load_modules([("TG_Cell", backup)]) else "ALSO FAILED - fix by hand"))
    return False


def load_probe():
    mark = tp.elog_mark()
    if tp.exec_state() != "stopped":
        tp.stop()
    with open(os.path.join(HERE, PROBE + ".mod"), "rb") as fh:
        tp.c.put_file("$home/TGS/%s.mod" % PROBE, fh.read())
    tp.c.request_mastership()
    try:
        if PROBE in module_names():
            tp.post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": PROBE})
            time.sleep(1.0)
        tp.post("/rw/rapid/tasks/T_ROB1", {"action": "loadmod"}, {"modulepath": "$home/TGS/%s.mod" % PROBE})
    finally:
        tp.c.release_mastership()
    time.sleep(2.0)
    for e in tp.elog_after(mark):
        log("  elog [%s] %s | %s" % e[1:])
    return build_ok()


def unload_probe():
    if tp.exec_state() != "stopped":
        tp.stop()
    tp.c.request_mastership()
    try:
        tp.post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": PROBE})
    finally:
        tp.c.release_mastership()
    log("probe module unloaded")


def run_routine(routine, fix=None, expect_code=None, timeout=90.0):
    """PP to routine, cycle once. With a fix: when TG_TouchSearch pauses for expect_code, write
    the fix PERS and press Start (once)."""
    mark = tp.elog_mark()
    if tp.exec_state() != "stopped":
        tp.stop()
    tp.c.request_mastership()
    try:
        tp.post("/rw/rapid/tasks/T_ROB1/pcp", {"action": "set-pp-routine"},
                {"module": PROBE, "routine": routine, "userlevel": "FALSE"})
    finally:
        tp.c.release_mastership()
    time.sleep(0.5)
    tp.start("once")
    fixed = False
    t = time.perf_counter()
    time.sleep(0.5)
    while time.perf_counter() - t < timeout:
        if tp.exec_state() == "stopped":
            step = sym(PROBE, "stTspStep").strip('"')
            if fix and not fixed and step.endswith("searching"):
                code = sym("TG_Touch", "nTG_TouchLastPause")
                log("  %s paused, reason %s - operator: %s.%s := %s, then Start" % (routine, code, fix[0], fix[1], fix[2]))
                if code != str(expect_code):
                    log("  (unexpected pause reason - not fixing)")
                    break
                set_sym(*fix)
                fixed = True
                time.sleep(1.0)
                tp.start("once")
                time.sleep(0.5)
                continue
            break
        time.sleep(0.2)
    raw = {k: sym(PROBE, k) for k in RESULTS}
    ev = tp.elog_after(mark)
    log("=== %s (%.1f s): step %s" % (routine, time.perf_counter() - t, raw["stTspStep"]))
    for e in ev:
        if e[1].startswith("800") or e[1] in ("40322", "40160", "40223", "41617", "40574", "40661"):
            log("  elog [%s] %s | %s" % (e[1], e[2].replace("\\n", " ").strip(), e[3].replace("\\n", " ").strip()[:120]))
    return raw, ev


def decode(raw):
    rt = {k: tpm.parse_robtarget(raw[v]) for k, v in (
        ("start", "pTspStartPart"), ("contact", "pTspContactPart"), ("hit", "pTspHit"),
        ("after", "pTspAfterPart"), ("after_w", "pTspAfterW"))}
    rt.update({"step": raw["stTspStep"].strip('"'), "face_z": float(raw["nTspFaceZ"]),
               "pauses": float(raw["nTspPauses"]), "last_pause": float(raw["nTspLastPause"]),
               "sensor_after": float(raw["nTspSensorAfter"]), "live_after": raw["bTspLiveAfter"] == "TRUE",
               "hit_ok": raw["bTspHitOK"] == "TRUE", "wobj": tpm.parse_wobj(raw["wobjTspPart"])})
    return rt


def main(argv):
    tp.connect()
    log("VC %s, execution %s, PP in %r" % (tp.URL, tp.exec_state(), tp.pcp_module()))
    if tp.exec_state() != "stopped" and tp.pcp_module() != "gapMain":
        log("REFUSED: PP in %r, not PM's idle gapMain - a cycle may be running" % tp.pcp_module())
        return 1
    titles = tp.signal_titles()
    if any(s not in titles for s in tp.SIM_SIGNALS):
        log("REFUSED: the World Zone rig signals are missing - run 'touch_probe.py setup-io' first")
        return 1
    if "--no-deploy" not in argv and not deploy():
        tp.back_to_pm()
        return 1
    if not load_probe():
        log("PROBE LOAD FAILED - see the event log above")
        unload_probe()
        tp.back_to_pm()
        return 1
    record, verdicts = {}, []
    before = {k: sym("TG_Cell", k) for k in ("stTG_TouchDI", "stTG_TouchOnDO", "stTG_TouchActiveDI", "stTG_TeachModeDO")}
    try:
        for i, (label, routine, pauses, code, fix) in enumerate(SCENARIOS):
            if i:
                time.sleep(PAUSE_S)
            raw, ev = run_routine(routine, fix, code)
            record[label] = {"raw": raw, "elog": ev}
            try:
                verdicts += tpm.p2_verdict(label, decode(raw), pauses, code)
            except ValueError as exc:
                verdicts.append(("%s results readable" % label, False, str(exc)))
    finally:
        out = os.path.join(HERE, time.strftime("touch_search_%Y%m%d_%H%M%S.json"))
        with open(out, "w") as fh:
            json.dump({"record": record, "verdicts": verdicts}, fh, indent=1, default=str)
        log("raw results: %s" % out)
        for step in (lambda: run_routine("TG_TspRestore"), lambda: run_routine("TG_TspHome"), unload_probe,
                     tp.back_to_pm):
            try:
                step()
            except Exception as exc:
                log("CLEAN-UP STEP FAILED: %s - finish it by hand (TG_TspRestore, park, unload, PP to main)" % exc)
        after = {k: sym("TG_Cell", k) for k in before}
        verdicts.append(("TG_Cell welder names restored", after == before, "%s" % after))
    log("")
    for name, ok, detail in verdicts:
        log("%s  %s -- %s" % ("PASS" if ok else "FAIL", name, detail))
    return 0 if verdicts and all(ok for _, ok, _ in verdicts) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
