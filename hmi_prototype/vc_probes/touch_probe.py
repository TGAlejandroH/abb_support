"""Touch-sense port P0 on the MONARC VC (docs/abb_touch_sense_port_v1.md §6, D4).

    python touch_probe.py setup-io     once: load TG_TouchSimEIO.cfg (VC-only signals), restart
    python touch_probe.py run          load TG_TouchProbe.mod, run X1 X2 X6 X3a X3b X3c, judge
    python touch_probe.py x7           the rig's resolution: the hit's sawtooth vs the face (P3)

``setup-io`` adds four virtual signals and two cross connections (additive, removal steps in
the cfg header), then warm-restarts the controller, because EIO changes need one.

``run`` refuses to start unless the program pointer sits in Production Manager's ``gapMain``.
That is the idle state: a TG cycle running under PM must never be interrupted (memory lesson
from the site comms test). It then works as the other VC probes do:
- one routine per experiment, PP-to-routine, cycle once;
- results are the probe's PERS values plus the new event-log entries, judged by
  touch_probe_math.py;
- the robot is parked, the probe unloaded and PM restarted from main.

X3c plays the operator: when the probe stops itself with stTpStep "X3c paused", this script
presses Start.

The raw results go to touch_probe_<timestamp>.json next to this file, so a run can be
re-judged later without the VC.
"""
import json
import os
import re
import sys
import time
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from rws_session import RwsSession  # noqa: E402
import touch_probe_math as tpm  # noqa: E402

URL = os.environ.get("TG_VC_RWS_URL", "http://127.0.0.1:80")
MODULE = "TG_TouchProbe"
MOD_PATH = os.path.join(HERE, MODULE + ".mod")
CFG_NAME = "TG_TouchSimEIO.cfg"
CFG_PATH = os.path.join(HERE, CFG_NAME)
SIM_SIGNALS = ("doTG_SimTouch", "diTG_SimTouched", "doTG_SimSensorOn", "diTG_SimSensorActive")
H = {"Content-Type": "application/x-www-form-urlencoded"}
RESULTS = ("stTpStep", "nTpErrno", "nTpFaceZ", "nTpSearchS", "nTpRetries", "nTpDiAtStart",
           "nTpFrameErr", "nTpOffLine", "nTpHitZ", "nTpStopZ", "pTpStart", "pTpTo", "pTpHit",
           "pTpStop", "pTpErrPos", "pTpStartPart", "pTpToPart", "pTpHitPart", "pTpStopPart",
           "pTpStopW", "wobjTpPart")
T0 = time.perf_counter()
c = None


def log(msg):
    print("[%7.2f] %s" % (time.perf_counter() - T0, msg), flush=True)


def connect():
    global c
    c = RwsSession(URL)


def get(p, query=None):
    q = {"json": "1", "lang": "en"}
    q.update(query or {})
    return c._request("GET", p, query=q).decode("utf-8", "replace")


def post(p, q, f):
    return c._request("POST", p, data=urllib.parse.urlencode(f).encode(), query=q, headers=H)


# ---------------------------------------------------------------- RWS plumbing (as listen_probe.py)
def elog_mark():
    return max(int(x) for x in re.findall(r"/rw/elog/0/(\d+)", get("/rw/elog/0")))


def elog_after(mark):
    b = get("/rw/elog/0")
    out = []
    for m in re.finditer(r'"_title":"/rw/elog/0/(\d+)"(.*?)(?="_title":"/rw/elog/0/|\Z)', b, re.S):
        if int(m.group(1)) <= mark:
            continue
        seg = m.group(2)
        g = lambda p: (re.search(p, seg, re.S).group(1) if re.search(p, seg, re.S) else "")
        desc = re.sub(r"\s+", " ", g(r'"desc":"([^"]*)"')).strip()
        out.append((int(m.group(1)), g(r'"code":"(\d+)"'), re.sub(r"\s+", " ", g(r'"title":"([^"]*)"')).strip(),
                    desc[:220]))
    return sorted(out)


def exec_state():
    return re.search(r'"ctrlexecstate":\s*"([^"]+)"', get("/rw/rapid/execution")).group(1)


def pcp_module():
    m = re.search(r'"modulemame":\s*"([^"]*)"', get("/rw/rapid/tasks/T_ROB1/pcp"))
    return m.group(1) if m else ""


def stop():
    post("/rw/rapid/execution", {"action": "stop"}, {"stopmode": "stop", "usetsp": "normal"})
    time.sleep(1.5)


def start(cycle, tries=4):
    # VC-observed 2026-09-25: a start issued ~0.5 s after the previous routine stopped itself
    # (after a Stop + restart) was refused with 400 once and accepted a second later.
    for attempt in range(tries):
        try:
            post("/rw/rapid/execution", {"action": "start"},
                 {"regain": "continue", "execmode": "continue", "cycle": cycle,
                  "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "false"})
            return
        except Exception as exc:
            if attempt == tries - 1:
                raise
            log("  start refused (%s) - retrying" % str(exc)[-40:])
            time.sleep(1.0)


def sym(name):
    body = get("/rw/rapid/symbol/data/RAPID/T_ROB1/%s/%s" % (MODULE, name))
    m = re.search(r'"value":\s*"((?:\\.|[^"\\])*)"', body)
    return m.group(1).replace('\\"', '"') if m else "?"


def back_to_pm():
    c.request_mastership()
    try:
        post("/rw/rapid/execution", {"action": "resetpp"}, {})
    finally:
        c.release_mastership()
    time.sleep(1.0)
    start("forever")
    time.sleep(4.0)
    log("back to Production Manager: execution %s" % exec_state())


def signal_titles():
    body = get("/rw/iosystem/signals", {"limit": "2000"})
    return {t.split("/")[-1]: t for t in re.findall(r'"_title":"([^"]+)"', body)}


# ---------------------------------------------------------------- setup-io
def setup_io():
    titles = signal_titles()
    missing = [s for s in SIM_SIGNALS if s not in titles]
    if not missing:
        log("VC-only signals already present: %s" % ", ".join(titles[s] for s in SIM_SIGNALS))
        return cross_check(titles)
    log("missing: %s -> loading %s" % (", ".join(missing), CFG_NAME))
    if exec_state() != "stopped":
        if pcp_module() != "gapMain":
            log("REFUSED: program pointer in %r, not PM's idle gapMain - a cycle may be running" % pcp_module())
            return 1
        stop()
    mark = elog_mark()
    with open(CFG_PATH, "rb") as fh:
        c.put_file("$home/TGS/" + CFG_NAME, fh.read())
    fields = {"filepath": "$home/TGS/" + CFG_NAME, "action-type": "add"}
    c.request_mastership()
    try:
        post("/rw/cfg", {"action": "validate"}, fields)
        log("validate: OK")
        post("/rw/cfg", {"action": "load"}, fields)
        log("load: OK")
    finally:
        c.release_mastership()
    for e in elog_after(mark):
        log("  elog [%s] %s | %s" % e[1:])
    log("warm restart (EIO changes take effect at restart)")
    try:
        # RW 6.15 VC: the restart form posts to /ctrl itself (action=""). /ctrl?action=restart
        # answers 400 - VC-observed 2026-09-25, and the developer-center page does not say which.
        post("/ctrl", {}, {"restart-mode": "restart"})
    except Exception as exc:                       # the connection may drop as the VC goes down
        log("  restart POST: %s" % str(exc)[-80:])
    time.sleep(10.0)
    deadline = time.time() + 240
    while time.time() < deadline:
        try:
            connect()
            state = re.search(r'"ctrlstate":\s*"([^"]+)"', get("/rw/panel/ctrlstate")).group(1)
            log("controller back: %s, execution %s" % (state, exec_state()))
            break
        except Exception:
            time.sleep(3.0)
    else:
        log("controller did not come back within 240 s")
        return 1
    titles = signal_titles()
    missing = [s for s in SIM_SIGNALS if s not in titles]
    if missing:
        log("FAIL: still missing after restart: %s" % ", ".join(missing))
        return 1
    rc = cross_check(titles)
    back_to_pm()
    return rc


def set_signal(title, value):
    post("/rw/iosystem/signals/" + title, {"action": "set"}, {"lvalue": str(value)})


def read_signal(title):
    return re.search(r'"lvalue":\s*"([^"]*)"', get("/rw/iosystem/signals/" + title)).group(1)


def cross_check(titles):
    """Each DO must show up on its DI through the cross connection."""
    ok = True
    for do, di in (("doTG_SimTouch", "diTG_SimTouched"), ("doTG_SimSensorOn", "diTG_SimSensorActive")):
        seen = []
        for v in (1, 0):
            set_signal(titles[do], v)
            time.sleep(0.3)
            seen.append(read_signal(titles[di]))
        good = seen == ["1", "0"]
        ok = ok and good
        log("%s  %s -> %s: DO 1/0 read back on the DI as %s" % ("PASS" if good else "FAIL", do, di, "/".join(seen)))
    return 0 if ok else 1


# ---------------------------------------------------------------- run
def load():
    mark = elog_mark()
    if exec_state() != "stopped":
        stop()
    with open(MOD_PATH, "rb") as fh:
        c.put_file("$home/TGS/" + MODULE + ".mod", fh.read())
    c.request_mastership()
    try:
        try:
            post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": MODULE})
        except Exception:
            pass
        time.sleep(1.0)
        post("/rw/rapid/tasks/T_ROB1", {"action": "loadmod"}, {"modulepath": "$home/TGS/" + MODULE + ".mod"})
    finally:
        c.release_mastership()
    time.sleep(2.0)
    ev = elog_after(mark)
    for e in ev:
        log("  elog [%s] %s | %s" % e[1:])
    return not any(e[1] in ("40322", "40160") for e in ev)


def unload():
    if exec_state() != "stopped":
        stop()
    c.request_mastership()
    try:
        post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": MODULE})
    finally:
        c.release_mastership()
    log("probe module unloaded")


def run_routine(routine, timeout=90.0, operator=False):
    """PP to routine, cycle once. operator=True presses Start again when the probe paused itself."""
    mark = elog_mark()
    if exec_state() != "stopped":
        stop()
    c.request_mastership()
    try:
        post("/rw/rapid/tasks/T_ROB1/pcp", {"action": "set-pp-routine"},
             {"module": MODULE, "routine": routine, "userlevel": "FALSE"})
    finally:
        c.release_mastership()
    time.sleep(0.5)
    t = time.perf_counter()
    start("once")
    paused_seen = False
    time.sleep(0.5)
    while time.perf_counter() - t < timeout:
        if exec_state() == "stopped":
            if operator and not paused_seen and sym("stTpStep").strip('"') == "X3c paused":
                paused_seen = True
                log("  %s paused itself (D3) - pressing Start as the operator" % routine)
                time.sleep(1.0)
                start("once")
                time.sleep(0.5)
                continue
            break
        time.sleep(0.2)
    raw = {k: sym(k) for k in RESULTS}
    ev = elog_after(mark)
    log("=== %s (%.1f s, %s): step %s" % (routine, time.perf_counter() - t, exec_state(), raw["stTpStep"]))
    for e in ev:
        log("  elog [%s] %s | %s" % e[1:])
    return raw, ev, paused_seen


def decode(raw):
    """RWS strings -> the dict the verdicts take."""
    rt = {k: tpm.parse_robtarget(raw[v]) for k, v in (
        ("start", "pTpStart"), ("to", "pTpTo"), ("hit", "pTpHit"), ("stop", "pTpStop"),
        ("err_pos", "pTpErrPos"), ("start_part", "pTpStartPart"), ("to_part", "pTpToPart"),
        ("hit_part", "pTpHitPart"), ("stop_part", "pTpStopPart"), ("stop_w", "pTpStopW"))}
    num = lambda k: float(raw[k])
    rt.update({
        "step": raw["stTpStep"].strip('"'), "errno": num("nTpErrno"), "face_z": num("nTpFaceZ"),
        "search_s": num("nTpSearchS"), "retries": num("nTpRetries"), "di_at_start": num("nTpDiAtStart"),
        "frame_err": num("nTpFrameErr"), "off_line": num("nTpOffLine"),
        "hit_z": tpm.numbers(raw["nTpHitZ"]), "stop_z": tpm.numbers(raw["nTpStopZ"]),
        "wobj": tpm.parse_wobj(raw["wobjTpPart"]),
    })
    return rt


def run():
    if exec_state() != "stopped" and pcp_module() != "gapMain":
        log("REFUSED: program pointer in %r, not PM's idle gapMain - a cycle may be running" % pcp_module())
        return 1
    titles = signal_titles()
    missing = [s for s in SIM_SIGNALS if s not in titles]
    if missing:
        log("REFUSED: VC-only signals missing (%s) - run 'setup-io' first" % ", ".join(missing))
        return 1
    if not load():
        # Unload first: with semantic errors in the task, PP-to-main (resetpp) is refused (400).
        log("LOAD FAILED - see the event log above; nothing run")
        unload()
        back_to_pm()
        return 1
    log("probe module loaded clean")
    record, verdicts = {}, []
    try:
        for name, routine, operator, timeout in (("X1", "TG_TpX1", False, 90), ("X2", "TG_TpX2", False, 60),
                                                 ("X6", "TG_TpX6", False, 60), ("X3a", "TG_TpX3Miss", False, 60),
                                                 ("X3b", "TG_TpX3Sig", False, 60),
                                                 ("X3c", "TG_TpX3Pause", True, 90)):
            raw, ev, paused = run_routine(routine, timeout=timeout, operator=operator)
            record[name] = {"raw": raw, "elog": ev, "paused_seen": paused}
            try:
                r = decode(raw)
            except ValueError as exc:
                verdicts.append(("%s results readable" % name, False, str(exc)))
                continue
            record[name]["decoded"] = r
            if name == "X1":
                verdicts += tpm.x1_verdict(r)
            elif name == "X2":
                x1 = record["X1"].get("decoded")
                verdicts += tpm.x2_verdict(r, sum(x1["hit_z"]) / 3 if x1 else float("nan"))
            elif name == "X6":
                x1 = record["X1"].get("decoded")
                over = (sum(h - s for h, s in zip(x1["hit_z"], x1["stop_z"])) / 3) if x1 else float("nan")
                verdicts += tpm.x6_verdict(r, over)
            elif name == "X3a":
                verdicts += tpm.x3a_verdict(r)
            elif name == "X3b":
                verdicts += tpm.x3b_verdict(r)
            else:
                verdicts += tpm.x3c_verdict(r, paused)
    finally:
        # Results first: a failing clean-up step must never cost the data (it did once).
        out = os.path.join(HERE, time.strftime("touch_probe_%Y%m%d_%H%M%S.json"))
        with open(out, "w") as fh:
            json.dump({"record": record, "verdicts": verdicts}, fh, indent=1, default=str)
        log("raw results: %s" % out)
        for step in (lambda: run_routine("TG_TpHome"), unload, back_to_pm):
            try:
                step()
            except Exception as exc:
                log("CLEAN-UP STEP FAILED: %s - finish it by hand (park, unload, PP to main, start)" % exc)
    log("")
    for name, ok, detail in verdicts:
        log("%s  %s -- %s" % ("PASS" if ok else "FAIL", name, detail))
    return 0 if verdicts and all(ok for _, ok, _ in verdicts) else 1


def x7():
    """The rig's resolution (P3): twelve searches, the face 0.03 mm further each time."""
    if exec_state() != "stopped" and pcp_module() != "gapMain":
        log("REFUSED: program pointer in %r, not PM's idle gapMain" % pcp_module())
        return 1
    if not load():
        log("LOAD FAILED")
        unload()
        back_to_pm()
        return 1
    try:
        raw, _ev, _ = run_routine("TG_TpX7", timeout=150)
        depth = tpm.numbers(sym("nTpDepth"))
        log("X7 step %s" % raw["stTpStep"])
        log("face - hit per 0.03 mm face step (mm): %s" % ", ".join("%.3f" % d for d in depth))
        log("spread %.3f mm = %.1f ms at %.0f mm/s" % (max(depth) - min(depth),
                                                     1000 * (max(depth) - min(depth)) / tpm.SEARCH_SPEED,
                                                     tpm.SEARCH_SPEED))
    finally:
        run_routine("TG_TpHome")
        unload()
        back_to_pm()
    return 0


def main(argv):
    connect()
    log("VC %s, execution %s, PP in %r" % (URL, exec_state(), pcp_module()))
    if argv[:1] == ["setup-io"]:
        return setup_io()
    if argv[:1] == ["run"]:
        return run()
    if argv[:1] == ["x7"]:
        return x7()
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
