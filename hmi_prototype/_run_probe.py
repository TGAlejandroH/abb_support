"""Run VC probe routines one at a time (PP to routine), then hand the cell back to PM.

    python _run_probe.py [--load vc_probes/TG_UfmecProbe.mod] TG_PrbUnbound TG_PrbBind ...

Each routine runs alone with cycle=once, so an unhandled RAPID error stops only that check and
lands in the event log. After each: the probe's PERS results and the new event-log entries.
At the end: reset PP to main and start, so Production Manager's EE_START hooks re-derive the
station in-position signals (TG_RoboCal L8/L9 recipe; socket plan lessons V-2/V-25/V-27).
"""
import os
import re
import sys
import time
import urllib.parse

from rws_session import RwsSession

URL = "http://127.0.0.1:80"
H = {"Content-Type": "application/x-www-form-urlencoded"}
MODULE = "TG_UfmecProbe"
RESULTS = ("stPrbStep", "stPrbUfmec", "nPrbStn", "pPrbS0", "pPrbS1", "pPrbW0", "pPrbW1")

c = RwsSession(URL)
get = lambda p: c._request("GET", p, query={"json": "1", "lang": "en"}).decode("utf-8", "replace")
post = lambda p, q, f: c._request("POST", p, data=urllib.parse.urlencode(f).encode(), query=q, headers=H)


def elog_after(mark):
    b = get("/rw/elog/0")
    out = []
    for m in re.finditer(r'"_title":"/rw/elog/0/(\d+)"(.*?)(?="_title":"/rw/elog/0/|\Z)', b, re.S):
        if int(m.group(1)) <= mark:
            continue
        seg = m.group(2)
        g = lambda p: (re.search(p, seg, re.S).group(1) if re.search(p, seg, re.S) else "")
        desc = re.sub(r"\s+", " ", g(r'"desc":"([^"]*)"')).strip()
        out.append((int(m.group(1)), g(r'"code":"(\d+)"'), re.sub(r"\s+", " ", g(r'"title":"([^"]*)"')).strip(), desc[:220]))
    return sorted(out)


def elog_mark():
    return max(int(x) for x in re.findall(r"/rw/elog/0/(\d+)", get("/rw/elog/0")))


def exec_state():
    return re.search(r'"ctrlexecstate":\s*"([^"]+)"', get("/rw/rapid/execution")).group(1)


def stop():
    post("/rw/rapid/execution", {"action": "stop"}, {"stopmode": "stop", "usetsp": "normal"})
    time.sleep(1.5)


def start(cycle):
    post("/rw/rapid/execution", {"action": "start"},
         {"regain": "continue", "execmode": "continue", "cycle": cycle,
          "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "false"})


def sym(name):
    # RAPID strings come back quoted and escaped ("value":"\"T1 start\""): allow escapes.
    m = re.search(r'"value":\s*"((?:\\.|[^"\\])*)"', get("/rw/rapid/symbol/data/RAPID/T_ROB1/%s/%s" % (MODULE, name)))
    return m.group(1).replace('\\"', '"') if m else "?"


def load(path):
    mark = elog_mark()
    stop()
    name = os.path.basename(path)
    with open(path, "rb") as fh:
        c.put_file("$home/TGS/" + name, fh.read())
    c.request_mastership()
    try:
        try:
            post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": os.path.splitext(name)[0]})
        except Exception as exc:
            print("  (unload: %s)" % str(exc)[-40:])
        time.sleep(1.0)
        post("/rw/rapid/tasks/T_ROB1", {"action": "loadmod"}, {"modulepath": "$home/TGS/" + name})
    finally:
        c.release_mastership()
    time.sleep(2.0)
    ev = elog_after(mark)
    for e in ev:
        print("  elog [%s] %s | %s" % e[1:])
    if any(e[1] == "40322" for e in ev):
        print("LOAD VERDICT: SYNTAX ERROR - %s did not load" % name)
        return False
    print("LOAD VERDICT: %s loaded clean (no 40322)" % name)
    return True


def run_routine(routine, timeout=90.0):
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
    t0 = time.time()
    start("once")
    time.sleep(1.0)
    while exec_state() != "stopped" and time.time() - t0 < timeout:
        time.sleep(0.5)
    print("\n=== %s  (%.1f s, %s)" % (routine, time.time() - t0, exec_state()))
    for r in RESULTS:
        print("  %-10s = %s" % (r, sym(r)))
    for e in elog_after(mark):
        print("  elog [%s] %s | %s" % e[1:])


def back_to_pm():
    c.request_mastership()
    try:
        post("/rw/rapid/execution", {"action": "resetpp"}, {})
    finally:
        c.release_mastership()
    time.sleep(1.0)
    start("forever")
    time.sleep(4.0)
    print("\nback to PM: execution %s" % exec_state())


def main(argv):
    if "--load" in argv:
        i = argv.index("--load")
        path = argv[i + 1]
        del argv[i:i + 2]
        if not load(path):
            return 1
    stay = "--stay" in argv
    routines = [a for a in argv if not a.startswith("--")]
    try:
        for routine in routines:
            run_routine(routine)
    finally:
        if not stay:
            # A check may stop with the robot at the part (T3 does, by design): park it first,
            # or PM's EE_START GoSafe waits on a pendant dialog.
            run_routine("TG_PrbHome")
            back_to_pm()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
