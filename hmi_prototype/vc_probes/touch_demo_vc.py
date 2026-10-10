"""Touch-sense demo on the MONARC VC: ONE TD05Touch cycle against a moved block, TCP traced.

    python touch_demo_vc.py [--shift DX DZ] [--speed PCT] [--out trace.json]

The same cycle as ``touch_real_vc.py`` (robotstudio_setup.md §22): TG_Main runs
``TGS/TD05Touch.mod`` for real, the production ``TG_TouchSearch`` searches the World Zone block
of ``TG_TrRig.mod``, and the prototype HMI computes the auto touch-up. Only one cycle, with the
block moved far enough to see (default +8 mm part X, -5 mm part Z), so it takes about 40 s.
``--speed 30`` runs the cycle at a 30 % speed override, so the 775 mm/s approaches can be
followed in the station view; the override is put back afterwards.

While the cycle runs, a second RWS session polls the TCP (``tTG_Weld`` in world) about every
0.1 s, and every HMI log line is time-stamped. Both go to the trace JSON with the verdict rows,
for a picture of what happened: the block is invisible in RobotStudio.

Same prerequisites and clean-up as ``touch_real_vc.py``.
"""
import argparse
import json
import os
import re
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import touch_probe as tp  # noqa: E402
import touch_real_vc as tr  # noqa: E402
from abb_server import TOUCH_REAL_DEMO  # noqa: E402
from rws_session import RwsSession  # noqa: E402
from touch_search_vc import module_names, sym  # noqa: E402

TCP_PATH = "/rw/motionsystem/mechunits/ROB_1/robtarget"
TCP_QUERY = {"tool": "tTG_Weld", "wobj": "wobj0", "coordinate": "Wobj", "json": "1"}
T0 = time.perf_counter()


def now():
    return time.perf_counter() - T0


class TcpSampler(threading.Thread):
    """Polls the TCP on its own RWS session until stopped; samples are (t, x, y, z)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.rws = RwsSession(tp.URL)
        self.samples = []
        self.halt = threading.Event()

    def run(self):
        while not self.halt.is_set():
            try:
                raw = self.rws._request("GET", TCP_PATH, query=TCP_QUERY).decode("utf-8", "replace")
                xyz = [float(re.search(r'"%s":"([^"]*)"' % k, raw).group(1)) for k in "xyz"]
                self.samples.append((round(now(), 3), *xyz))
            except Exception:
                time.sleep(0.2)


EVENTS = []


class DemoHmi(tr.LoggingHmi):
    def _log(self, msg):
        EVENTS.append((round(now(), 3), msg))
        super()._log(msg)


def speed_ratio():
    return int(re.search(r'"speedratio":"(\d+)"', tp.get("/rw/panel/speedratio")).group(1))


def set_speed_ratio(pct):
    tp.post("/rw/panel/speedratio", {"action": "setspeedratio"}, {"speed-ratio": str(pct)})


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--shift", nargs=2, type=float, default=(8.0, -5.0), metavar=("DX", "DZ"),
                    help="block shift in part X and Z, mm (default 8 -5)")
    ap.add_argument("--speed", type=int, default=100, metavar="PCT",
                    help="speed override for the cycle, 1-100 %% (default 100; 30 is easy to watch)")
    ap.add_argument("--out", default=os.path.join(HERE, time.strftime("touch_demo_%Y%m%d_%H%M%S.json")))
    args = ap.parse_args(argv)
    shift = (args.shift[0], 0.0, args.shift[1])

    tp.connect()
    tp.log("VC %s, execution %s, PP in %r" % (tp.URL, tp.exec_state(), tp.pcp_module()))
    if tp.exec_state() != "stopped" and tp.pcp_module() != "gapMain":
        tp.log("REFUSED: PP in %r, not PM's idle gapMain - a cycle may be running" % tp.pcp_module())
        return 1
    at = [re.search(r'"lvalue":"([^"]*)"', tp.get("/rw/iosystem/signals/Local/B_GAP_SIM/siGap_AtStn_%d" % n)).group(1)
          for n in (1, 2)]
    if sorted(at) != ["0", "1"]:
        tp.log("REFUSED: siGap_AtStn_1/2 = %s - TG_ActMechUnit needs exactly one station in position" % at)
        return 1
    if "TG_Touch" not in module_names():
        tp.log("REFUSED: TG_Touch is not loaded - run touch_search_vc.py (P2 deploy) first")
        return 1
    if not tr.load_rig():
        tp.log("RIG LOAD FAILED")
        return 1

    tr.LoggingHmi = DemoHmi          # run_cycle builds its HMI from this name
    sampler = TcpSampler()
    rows, result = [], None
    speed_before = speed_ratio()
    try:
        if args.speed != speed_before:
            set_speed_ratio(args.speed)
            tp.log("speed override %d %% -> %d %%" % (speed_before, speed_ratio()))
        sampler.start()
        EVENTS.append((round(now(), 3), "cycle start, block shift %s, speed %d %%" % (shift, args.speed)))
        result = tr.run_cycle("demo", shift)
        EVENTS.append((round(now(), 3), "cycle stopped"))
        tr.judge("demo", shift, result, rows)
    finally:
        sampler.halt.set()
        sampler.join(2.0)
        if speed_ratio() != speed_before:
            set_speed_ratio(speed_before)
            tp.log("speed override back to %d %%" % speed_ratio())
        for step in (lambda: tr.run_quick("TG_TrRestore"), lambda: tr.run_quick("TG_TrHome"), tr.unload_rig,
                     tp.back_to_pm):
            try:
                step()
            except Exception as exc:
                tp.log("CLEAN-UP STEP FAILED: %s - finish by hand (TG_TrRestore, park, unload TG_TrRig, PP to main)"
                       % exc)
        names = {k: sym("TG_Cell", k) for k in ("stTG_TouchDI", "stTG_TouchOnDO", "stTG_TouchActiveDI")}
        rows.append(("TG_Cell welder names restored", names == {"stTG_TouchDI": '"diWld1Touched"',
                     "stTG_TouchOnDO": '"doWld1TouchOn"', "stTG_TouchActiveDI": '"diWld1TouchActive"'}, "%s" % names))
        with open(args.out, "w") as fh:
            json.dump({"shift": shift, "speed": args.speed, "demo": TOUCH_REAL_DEMO, "samples": sampler.samples, "events": EVENTS,
                       "result": result, "rows": rows}, fh, indent=1, default=str)
        tp.log("trace: %s (%d TCP samples)" % (args.out, len(sampler.samples)))
    tp.log("")
    for name, ok, detail in rows:
        tp.log("%s  %s -- %s" % ("PASS" if ok else "FAIL", name, detail))
    return 0 if rows and all(ok for _, ok, _ in rows) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
