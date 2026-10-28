"""One VC case with the real HMI in the loop: record a part run and check it.

    python hmi_case.py --label F4 --expect expect.json [--part 9022] [--profile ...]
                       [--out-dir results] [--max-seconds 600]

The HMI must already be armed in socket-start mode with the work selection map pointing the part
at the fixture (hmi_case.ps1 does that). This starts the recorder, then the robot's
TG_Part<part> over RWS (bench_cycle.run_part: VC only), and checks the recording against
``expect.json`` - the planner's seam lengths and what the HMI should send for each weld. Nothing
here talks to the HMI, so what the robot ran with is compared with what the project says, not
with what the HMI says it sent.

Writes <out-dir>/<label>.json and <label>.txt. Exit code 0 = every check passed.
"""

import argparse
import json
import os
import sys
import threading
import time

from bench_cycle import run_part
from checks import check, format_rows
from recorder import DEFAULT_PROFILE, HERE, Recorder, load_profile
from report import summary_lines


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--expect", required=True)
    ap.add_argument("--part", default="9022")
    ap.add_argument("--profile", default=DEFAULT_PROFILE)
    ap.add_argument("--out-dir", default=os.path.join(HERE, "results"))
    ap.add_argument("--max-seconds", type=float, default=600.0)
    args = ap.parse_args(argv)
    os.makedirs(args.out_dir, exist_ok=True)
    with open(args.expect) as fh:
        expect = json.load(fh)

    result = {}

    def robot():
        try:
            result["task_state"] = run_part(args.part, args.max_seconds, label="hmi_case")
        except Exception as exc:
            result["error"] = repr(exc)

    rec = Recorder(load_profile(args.profile), args.max_seconds)
    runner = threading.Thread(target=robot, daemon=True)
    recording = rec.record(on_ready=lambda: (time.sleep(1.0), runner.start()))
    runner.join(30.0)
    recording["label"] = args.label
    recording["sent"] = {"source": "HMI in the loop - expected values from the fixture",
                         "task_state": result.get("task_state"), "error": result.get("error")}
    recording["expect"] = expect
    rows = check(recording, recording["sent"], expect)
    lines = ["case %s: part %s, HMI in the loop" % (args.label, args.part)] + summary_lines(recording) \
        + [""] + format_rows(rows)
    with open(os.path.join(args.out_dir, args.label + ".json"), "w") as fh:
        json.dump(recording, fh, indent=1)
    with open(os.path.join(args.out_dir, args.label + ".txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0 if all(ok is not False for _, ok, _ in rows) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
