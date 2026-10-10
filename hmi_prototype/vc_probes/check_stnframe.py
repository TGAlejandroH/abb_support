"""Stage R2 V-45 helper: the live wobjTG_WeldActStn binding, the deployed TG_Cell.sys binding
lines, recent event-log entries, and the handshake STNFRAME files against the T2 probe frames.

    set TG_VC_RWS_URL=http://127.0.0.1:53450
    python vc_probes/check_stnframe.py
"""
import glob
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
from probe_math import quat_to_R, station_frame  # noqa: E402
from rws_session import RwsSession  # noqa: E402

URL = os.environ.get("TG_VC_RWS_URL", "http://127.0.0.1:80")
VALUE_RE = re.compile(r'"value":\s*"((?:\\.|[^"\\])*)"')


def pose_matrix(literal):
    t, q = json.loads(literal)
    m = np.eye(4)
    m[:3, :3] = quat_to_R(np.array(q, float))
    m[:3, 3] = t
    return m


def main():
    c = RwsSession(URL)
    get = lambda p: c._request("GET", p, query={"json": "1", "lang": "en"}).decode("utf-8", "replace")

    print("---- live binding")
    b = get("/rw/rapid/symbol/data/RAPID/T_ROB1/TG_Comms/wobjTG_WeldActStn")
    m = VALUE_RE.search(b)
    print("wobjTG_WeldActStn =", m.group(1).replace('\\"', '"') if m else b[:200])

    print("---- deployed TG_Cell.sys lines mentioning ufmec")
    txt = c._request("GET", "/fileservice/$home/TGS/TG_Cell.sys").decode("utf-8", "replace")
    for i, line in enumerate(txt.splitlines()):
        if "ufmec" in line and not line.strip().startswith("!"):
            print("  %d: %s" % (i + 1, line.strip()))

    print("---- event log, last 10")
    b = get("/rw/elog/0")
    ids = sorted(int(x) for x in re.findall(r"/rw/elog/0/(\d+)", b))[-10:]
    for i in ids:
        d = get("/rw/elog/0/%d" % i)
        ts = re.search(r'"tstamp":\s*"([^"]*)"', d)
        code = re.search(r'"code":\s*"([^"]*)"', d)
        ti = re.search(r'"title":\s*"([^"]*)"', d)
        print("  [%s] %s %s" % (ts.group(1)[-8:] if ts else "?", code.group(1) if code else "?",
                                 (ti.group(1) if ti else "?")[:70]))

    np.set_printoptions(precision=3, suppress=True)
    print("---- T2 probe station frames (world <- station), at the probe's chuck angle")
    probes = {}
    for stn in (1, 2):
        d = json.load(open(os.path.join(HERE, "stn%d_T2.json" % stn)))
        f0 = station_frame(d["W0"], d["S0"])
        probes[stn] = (f0, d["c0"])
        print("  station %d at chuck %6.2f: origin %s  x-axis %s  z-axis %s"
              % (stn, d["c0"], f0[:3, 3], f0[:3, 0], f0[:3, 2]))

    print("---- handshake STNFRAME files")
    run_dir = os.path.join(os.path.dirname(HERE), "tgs_run")
    for path in sorted(glob.glob(os.path.join(run_dir, "stnframe_seq*_stn*.json"))):
        d = json.load(open(path))
        f = pose_matrix(d["station_frame"])
        stn = d["station"]
        line = "  seq %d station %d tilt %s chuck %s: origin %s  x-axis %s  z-axis %s" % (
            d["seq"], stn, d["tilt_deg"], d["chuck_deg"], f[:3, 3], f[:3, 0], f[:3, 2])
        if stn in probes:
            f0, c0 = probes[stn]
            dt = np.linalg.norm(f[:3, 3] - f0[:3, 3])
            rel = f0[:3, :3].T @ f[:3, :3]
            ang = np.degrees(np.arccos(np.clip((np.trace(rel) - 1) / 2, -1, 1)))
            line += "\n      vs probe: origin differs %.3f mm, rotation differs %.3f deg (probe chuck %.2f vs %s)" % (
                dt, ang, c0, d["chuck_deg"])
        print(line)


if __name__ == "__main__":
    main()
