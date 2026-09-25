"""Observe the VC's motion system through a dispatch - the index check.

Question this answers (socket_start_trigger_hmi_plan_v1.md, V-16 retraction):
does Production Manager actually index the turntable, and does its index
pre-position the chucks from partadv? Reading Irbp1EEv.sys says yes -
EE_INDEX -> EvIndexToStn1 -> ActInterch1; IndexToStn1; DeactInterch1 - but
the earlier V-10 probe read the chuck at 0 where partadv asked for 30, so the
code reading and the measurement disagree. This settles it by watching.

What an index looks like from outside: INTERCH goes Activated, the program
pointer enters Irbp1Prc/IndexToStn<n>, the index axis (eax_d while INTERCH is
the active unit) moves ~180 deg, INTERCH goes Deactivated. Then EE_PRE_PART's
ActStn<n> activates the station. Sampling only at START (as V-10 did) sees
none of that, because by then INTERCH is already deactivated again.

Logs a line on any change of: execution state, program pointer, unit modes;
or when a joint moved more than MOVE_DEG since the last logged line.
One persistent RWS session (rws_session.RwsSession) - the V-3 fix - so a
4 Hz poll does not exhaust the controller's session pool.

Usage: python vc_monitor.py [period_s]     Ctrl-C to stop.
"""

import re
import sys
import time

from rws_session import RwsSession

import os
URL = os.environ.get("TG_VC_RWS_URL", "http://127.0.0.1:80")   # the VC is not always on 80
MOVE_DEG = 2.0

AX = re.compile(r'"(rax_\d|eax_[a-f])"\s*:\s*"?([-0-9.eE+]+)"?')


def ts():
    return time.strftime("%H:%M:%S") + ".%03d" % int((time.time() % 1) * 1000)


def get(c, path):
    return c._request("GET", path, query={"json": "1"}).decode("utf-8", "replace")


def modes(c):
    b = get(c, "/rw/motionsystem/mechunits")
    names = re.findall(r'"_title"\s*:\s*"([^"]+)"', b)
    ms = re.findall(r'"mode"\s*:\s*"([^"]+)"', b)
    return dict(zip(names, ms))


def joints(c, unit):
    vals = dict(AX.findall(get(c, "/rw/motionsystem/mechunits/%s/jointtarget" % unit)))
    out = {}
    for k, v in vals.items():
        f = float(v)
        if abs(f) < 1e8:           # drop 9E9 "not applicable" slots
            out[k] = round(f, 1)
    return out


def pcp(c):
    b = get(c, "/rw/rapid/tasks/T_ROB1/pcp")
    m = re.search(r'"modulemame"\s*:\s*"([^"]*)"', b)
    r = re.search(r'"routinename"\s*:\s*"([^"]*)"', b)
    return "%s/%s" % (m.group(1) if m else "?", r.group(1) if r else "?")


def execstate(c):
    m = re.search(r'"ctrlexecstate"\s*:\s*"([^"]+)"', get(c, "/rw/rapid/execution"))
    return m.group(1) if m else "?"


def fmt_modes(md):
    short = {"ROB_1": "ROB", "INTERCH": "INT", "STN1": "S1", "STN2": "S2"}
    return " ".join("%s%s" % (short.get(k, k), "+" if v == "Activated" else "-")
                    for k, v in md.items())


def moved(a, b):
    if a is None or b is None or a.keys() != b.keys():
        return True
    return any(abs(a[k] - b[k]) > MOVE_DEG for k in a)


def main(argv):
    period = float(argv[1]) if len(argv) > 1 else 0.25
    c = RwsSession(URL)
    last_key = None
    last_axes = None
    print("[MON %s] monitor started, period %.2fs, log on change or >%.0f deg move"
          % (ts(), period, MOVE_DEG), flush=True)
    while True:
        try:
            md = modes(c)
            ex = execstate(c)
            pp = pcp(c)
            axes = {"ROB": joints(c, "ROB_1")}
            for unit in ("INTERCH", "STN1", "STN2"):
                if md.get(unit) == "Activated":
                    axes[unit] = joints(c, unit)
            flat = {"%s.%s" % (u, k): v for u, d in axes.items() for k, v in d.items()}
            key = (ex, pp, tuple(sorted(md.items())))
            if key != last_key or moved(flat, last_axes):
                parts = []
                for unit, d in axes.items():
                    if unit == "ROB":
                        rob = [d.get("rax_%d" % i) for i in range(1, 7)]
                        ext = {k: v for k, v in d.items() if k.startswith("eax") and v != 0}
                        parts.append("ROB[%s]%s" % (",".join("%g" % x for x in rob if x is not None),
                                                   (" ext" + str(ext)) if ext else ""))
                    else:
                        parts.append("%s%s" % (unit, d))
                print("[MON %s] %-8s %-26s %-18s %s" % (ts(), ex, pp, fmt_modes(md), " ".join(parts)),
                      flush=True)
                last_key, last_axes = key, flat
        except Exception as exc:
            print("[MON %s] poll error: %s" % (ts(), exc), flush=True)
            time.sleep(1.0)
            c = RwsSession(URL)
        time.sleep(period)


if __name__ == "__main__":
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print()
