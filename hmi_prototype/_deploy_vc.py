"""Stop RAPID, reload the TG modules, verify via the event log, restart.

One RwsSession throughout (V-3). The event log is the verdict, not the POST
(V-2: loadmod reports success even for a module that fails to compile).
"""
import os, re, time, urllib.parse
from rws_session import RwsSession

RAPID = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "abb", "rapid")
# TG_Touch.sys (touch-sense P2) after TG_Cell: it calls TG_Cell's touch macros.
MODULES = ["TG_Comms.sys", "TG_Cell.sys", "TG_Touch.sys", "TG_Weld.sys", "TG_Main.mod", "TG_Parts.mod"]
c = RwsSession("http://127.0.0.1:80")

def post(path, fields, query=None):
    body = urllib.parse.urlencode(fields).encode("ascii")
    return c._request("POST", path, data=body, query=query,
                      headers={"Content-Type": "application/x-www-form-urlencoded"})

def get(path):
    return c._request("GET", path, query={"json": "1", "lang": "en"}).decode("utf-8", "replace")

def exec_state():
    return re.search(r'"ctrlexecstate":\s*"([^"]+)"', get("/rw/rapid/execution")).group(1)

def elog_blocks():
    b = get("/rw/elog/0")
    for m in re.finditer(r'"_title":"/rw/elog/0/(\d+)"(.*?)(?="_title":"/rw/elog/0/|\Z)', b, re.S):
        seg = m.group(2)
        g = lambda p: (re.search(p, seg).group(1) if re.search(p, seg) else "")
        yield int(m.group(1)), g(r'"code":"(\d+)"'), g(r'"title":"([^"]*)"'), g(r'"desc":"([^"]*)"')

mark = max(i for i, *_ in elog_blocks())
print("elog mark", mark, "| exec before:", exec_state())

post("/rw/rapid/execution", {"stopmode": "stop", "usetsp": "normal"}, {"action": "stop"})
time.sleep(1.5)
print("exec after stop:", exec_state())

for m in MODULES:
    with open(os.path.join(RAPID, m), "rb") as fh:
        c.put_file("$home/TGS/" + m, fh.read())
print("uploaded", len(MODULES), "modules")

c.request_mastership()
try:
    # RWS 1.0 form: the action goes on the TASK resource (VC-verified 2026-09-22;
    # a /loadmod sub-path is 404).
    for m in reversed(MODULES):                  # unload dependents first
        try:
            post("/rw/rapid/tasks/T_ROB1", {"module": os.path.splitext(m)[0]}, {"action": "unloadmod"})
            print("  unloaded", m)
        except Exception as e:
            print("  (unload %s: %s)" % (m, str(e)[-40:]))
    time.sleep(1.0)
    for m in MODULES:
        post("/rw/rapid/tasks/T_ROB1", {"modulepath": "$home/TGS/" + m}, {"action": "loadmod"})
        print("  load POST ok", m)
finally:
    try:
        c.release_mastership()
    except Exception:
        pass
time.sleep(2.0)

bad = [(i, code, t, d) for i, code, t, d in elog_blocks() if i > mark and code in ("40322", "40223", "40320")]
print("load errors since mark:", "NONE" if not bad else "")
for i, code, t, d in bad:
    print("  !!", code, t, "|", d[:160])

for mod, sym in [("TG_Comms", "nTG_PartStn"), ("TG_Comms", "nTG_CycleSeq"),
                 ("TG_Parts", "padvTG_9011"), ("TG_Parts", "padvTG_9022")]:
    v = re.search(r'"value":\s*"([^"]*)"', get("/rw/rapid/symbol/data/RAPID/T_ROB1/%s/%s" % (mod, sym)))
    print("  %s.%s = %s" % (mod, sym, v.group(1) if v else "?"))
for mod, proc in [("TG_Parts", "TG_Part9022"), ("TG_Main", "TG_VisionOnce")]:
    try:
        get("/rw/rapid/symbol/properties/RAPID/T_ROB1/%s/%s" % (mod, proc)); print("  routine %s.%s resolves" % (mod, proc))
    except Exception as e:
        print("  routine %s.%s MISSING: %s" % (mod, proc, e))

post("/rw/rapid/execution", {"regain": "continue", "execmode": "continue", "cycle": "forever",
     "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "false"}, {"action": "start"})
time.sleep(3.0)
pp = get("/rw/rapid/tasks/T_ROB1/pcp")
print("exec after start:", exec_state(), "| PP:",
      re.search(r'"modulemame":\s*"([^"]*)"', pp).group(1), "/", re.search(r'"routinename":\s*"([^"]*)"', pp).group(1))
