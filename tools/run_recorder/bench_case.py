"""One VC test case in one command: record a bench cycle and check it.

    python bench_case.py --label E59_site --mod path/to/T9382dffx1.mod [--params params.json]
                         [--expect expect.json] [--part 9022] [--profile ...] [--out-dir results]

Starts the recorder, runs bench_cycle.run_cycle (stand-in HMI, VC only), then writes
<out-dir>/<label>.json (the recording plus what was served) and <label>.txt (summary and checks).
Exit code 0 = every check passed.
"""

import argparse
import json
import os
import sys
import threading
import time

from bench_cycle import run_cycle
from checks import check, format_rows
from recorder import DEFAULT_PROFILE, HERE, Recorder, load_profile
from report import summary_lines


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--mod", required=True)
    ap.add_argument("--params")
    ap.add_argument("--expect")
    ap.add_argument("--part", default="9022")
    ap.add_argument("--profile", default=DEFAULT_PROFILE)
    ap.add_argument("--out-dir", default=os.path.join(HERE, "results"))
    ap.add_argument("--max-seconds", type=float, default=600.0)
    args = ap.parse_args(argv)
    os.makedirs(args.out_dir, exist_ok=True)
    params = None
    if args.params:
        with open(args.params) as fh:
            params = json.load(fh)

    result = {}

    def bench():
        try:
            result["sent"] = run_cycle(args.mod, params, args.part, args.max_seconds)
        except Exception as exc:
            result["sent"] = {"error": repr(exc), "weld_params": []}

    rec = Recorder(load_profile(args.profile), args.max_seconds)
    runner = threading.Thread(target=bench, daemon=True)
    # The bench starts once the sampler is polling, so the recording covers the whole run.
    recording = rec.record(on_ready=lambda: (time.sleep(1.0), runner.start()))
    runner.join(30.0)
    recording["label"] = args.label
    recording["sent"] = result.get("sent")

    expect = None
    if args.expect:
        with open(args.expect) as fh:
            expect = json.load(fh)
    recording["expect"] = expect
    rows = check(recording, recording["sent"], expect)
    lines = ["case %s: module %s, part %s" % (args.label, args.mod, args.part)] + summary_lines(recording) \
        + [""] + format_rows(rows)
    with open(os.path.join(args.out_dir, args.label + ".json"), "w") as fh:
        json.dump(recording, fh, indent=1)
    with open(os.path.join(args.out_dir, args.label + ".txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0 if all(ok is not False for _, ok, _ in rows) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
