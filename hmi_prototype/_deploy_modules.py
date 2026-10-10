"""Deploy only the named TG modules to the VC, verified by the event log.

    python _deploy_modules.py TG_Cell.sys [TG_Comms.sys ...]

Why only the named ones: reloading a module from a file discards anything written into its
in-memory copy - PM's pendant teaching writes partadv into TG_Parts (plan V-30). Never reload a
module that did not change.

Steps (TG_RoboCal L8 recipe; lessons V-2/V-25/V-27): stop, upload, unload + load with one RWS
session (L9), then read the event log - the loadmod POST "succeeds" even on a syntax error -
then reset PP to main and start fresh so Production Manager's EE_START hooks re-derive the
station in-position signals.
"""
import os
import re
import sys
import time
import urllib.parse

from rws_session import RwsSession

RAPID = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "abb", "rapid")
URL = os.environ.get("TG_VC_RWS_URL", "http://127.0.0.1:80")   # the VC is not always on 80
H = {"Content-Type": "application/x-www-form-urlencoded"}


def main(modules):
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
            g = lambda p: (re.search(p, seg).group(1) if re.search(p, seg) else "")
            out.append((int(m.group(1)), g(r'"code":"(\d+)"'), g(r'"title":"([^"]*)"'), g(r'"desc":"([^"]*)"')[:170]))
        return sorted(out)

    mark = max(int(x) for x in re.findall(r"/rw/elog/0/(\d+)", get("/rw/elog/0")))
    post("/rw/rapid/execution", {"action": "stop"}, {"stopmode": "stop", "usetsp": "normal"})
    time.sleep(1.5)
    for m in modules:
        with open(os.path.join(RAPID, m), "rb") as fh:
            c.put_file("$home/TGS/" + m, fh.read())
        print("uploaded", m)
    c.request_mastership()
    try:
        for m in modules:
            try:
                post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": os.path.splitext(m)[0]})
            except Exception as exc:
                print("  (unload %s: %s)" % (m, str(exc)[-40:]))
        time.sleep(1.0)
        for m in modules:
            post("/rw/rapid/tasks/T_ROB1", {"action": "loadmod"}, {"modulepath": "$home/TGS/" + m})
            print("load POST ok", m, "(not a verdict - reading the event log)")
    finally:
        c.release_mastership()
    time.sleep(2.0)

    events = elog_after(mark)
    hard = [e for e in events if e[1] == "40322"]
    link = [e for e in events if e[1] == "40160"]
    for e in events:
        if e[1] in ("40322", "40160", "40223"):
            print("  !! [%s] %s | %s" % e[1:])
    if hard:
        print("VERDICT: SYNTAX ERROR - module did not load. Not starting.")
        return 1

    c.request_mastership()
    try:
        post("/rw/rapid/execution", {"action": "resetpp"}, {})
    finally:
        c.release_mastership()
    time.sleep(1.0)
    try:
        post("/rw/rapid/execution", {"action": "start"},
             {"regain": "continue", "execmode": "continue", "cycle": "forever",
              "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "false"})
    except Exception as exc:
        print("VERDICT: program will not start (%s) - unresolved references remain" % exc)
        return 1
    rd = lambda s: re.search(r'"lvalue":"([^"]*)"', get("/rw/iosystem/signals/Local/B_GAP_SIM/" + s)).group(1)
    for _ in range(10):
        time.sleep(1.0)
        st = tuple(rd(s) for s in ("siGap_AtStn_1", "siGap_AtStn_2", "siGap_NextStn_1", "siGap_NextStn_2"))
        if "1" in st[:2]:
            break
    ex = re.search(r'"ctrlexecstate":\s*"([^"]+)"', get("/rw/rapid/execution")).group(1)
    print("VERDICT: loaded clean%s; execution %s; AtStn_1=%s AtStn_2=%s NextStn_1=%s NextStn_2=%s"
          % ((" (transient 40160 cleared - program started)" if link else "", ex) + st))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:] or ["TG_Cell.sys"]))
