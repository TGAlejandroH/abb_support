"""Step 0 of the ABB production readiness plan (TGuideWeldingHMI docs/abb_production_readiness_plan_v1.md
section 5.3): prove, on a live run, that RWS supplies what the run recorder needs.

    python step0_probe.py [--profile profiles/...json] [--out step0.json] [--max-seconds 600]

Start it BEFORE the run; it records the run with recorder.Recorder and answers each probe:

  P1  weld data during the weld   wdTG_Weld read while the program pointer is on the span's
                                  welding line, equal to the value after the run
  P2  module LOCAL data           LOCAL data of the loaded weld module (its weave, when it has
                                  one) read while the module is loaded, equal to the declaration
  P3  sampling rate               edge uncertainty <= 0.1 s on every clock, enough to time a 2 s
                                  weld within 10 %
  P4  welder outputs              weld-on rises and falls around the span; the schedule and wire
                                  feed outputs follow the weld data
  P5  run not disturbed           no RWS read errors; the run ended on its own
  P6  the clocks agree            TCP travel time, weld-on time and planned length / weld_speed
                                  agree within 10 % (or 0.2 s)

Read-only: every request is a GET.
"""

import argparse
import json
import os
import sys
import time

from analysis import expected_outputs
from recorder import DEFAULT_PROFILE, HERE, Recorder, load_profile
from report import summary_lines

VERDICT = {True: "PASS", False: "FAIL", None: "NOT RUN"}


def _num(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def probes(recording):
    a = recording["analysis"]
    spans = [s for s in a["spans"] if s["observed"]]
    rows = []

    during = [s["welddata_seen"] for s in spans]
    p1 = None if not spans else all(during) and during[-1][-1] == recording["after"]["wdTG_Weld"]
    rows.append(("P1 weld data during the weld", p1, "per weld %s; after the run %s" % (during, recording["after"]["wdTG_Weld"])))

    reads = recording["module"].get("local_reads", [])
    p2 = None if not reads else all("value" in r for r in reads)
    rows.append(("P2 module LOCAL data while loaded", p2,
                 "; ".join("%s %s = %s (declared %s)" % (r.get("type"), r["name"], r.get("value", r.get("error")),
                                                         r.get("declared")) for r in reads)
                 + ("" if any(r.get("type") == "weavedata" for r in reads) else " [no weave in this module]")))

    unc = []
    for s in spans:
        unc += [s.get("edge_uncertainty"), s["weld_on"]["uncertainty"]]
    unc = [u for u in unc if u is not None]
    p3 = None if not unc else max(unc) <= 0.1
    rows.append(("P3 sampling rate", p3, "median tick %s s; edge uncertainties %s s"
                 % (a["sampling"].get("median_tick_s"), unc)))

    p4 = None
    detail = []
    if spans:
        p4 = True
        for s in spans:
            ok_on = s["weld_on"]["seconds"] is not None
            p4 = p4 and ok_on
            parts = ["weld %d: weld-on %s s" % (s["n"], s["weld_on"]["seconds"])]
            for name, want in expected_outputs(recording["profile"], s["welddata"]).items():
                seen = [_num(v) for v in s["outputs_seen"].get(name, [])]
                ok = want is not None and any(v is not None and abs(v - want) < 0.5 for v in seen)
                p4 = p4 and ok
                parts.append("%s %s (from the weld data: %s)" % (name, seen, want))
            detail.append(", ".join(parts))
    rows.append(("P4 welder outputs follow the weld", p4, "; ".join(detail)))

    p5 = recording["rws"]["read_errors"] == [] and recording["ended_because"] != "time limit or interrupt"
    rows.append(("P5 run not disturbed", p5, "%s RWS requests, read errors %s, ended because %s"
                 % (recording["rws"]["requests"], recording["rws"]["read_errors"][:3], recording["ended_because"])))

    p6 = None
    detail = []
    for s in spans:
        wd = s["welddata"] or {}
        if not wd.get("weld_speed") or s.get("travel_seconds") is None:
            continue
        expect = s["planned_mm"] / wd["weld_speed"]
        times = [s["travel_seconds"], s["weld_on"]["seconds"]]
        ok = all(t is not None and abs(t - expect) <= max(0.1 * expect, 0.2) for t in times)
        p6 = ok if p6 is None else (p6 and ok)
        detail.append("weld %d: planned %.1f mm / %.3f mm/s = %.2f s; TCP %s s; weld-on %s s"
                      % (s["n"], s["planned_mm"], wd["weld_speed"], expect, s["travel_seconds"], s["weld_on"]["seconds"]))
    rows.append(("P6 the clocks agree", p6, "; ".join(detail)))
    return rows


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default=DEFAULT_PROFILE)
    ap.add_argument("--out", default=os.path.join(HERE, "results", time.strftime("step0_%Y%m%d_%H%M%S.json")))
    ap.add_argument("--max-seconds", type=float, default=600.0)
    ap.add_argument("--reanalyse", help="re-run the probes on an existing step0 JSON instead of recording")
    args = ap.parse_args(argv)
    if args.reanalyse:
        with open(args.reanalyse) as fh:
            recording = json.load(fh)
        from analysis import analyse
        recording["analysis"] = analyse(recording)
    else:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        rec = Recorder(load_profile(args.profile), args.max_seconds)
        print("Step 0: waiting for the run (profile %s)" % rec.profile["name"], flush=True)
        recording = rec.record()
    rows = probes(recording)
    recording["step0"] = [{"probe": n, "verdict": VERDICT[v], "detail": d} for n, v, d in rows]
    if not args.reanalyse:
        with open(args.out, "w") as fh:
            json.dump(recording, fh, indent=1)
    for line in summary_lines(recording):
        print(line)
    print()
    for n, v, d in rows:
        print("%-7s %s -- %s" % (VERDICT[v], n, d))
    print("recording: %s" % (args.reanalyse or args.out))
    return 0 if all(v is not False for _, v, _ in rows) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
