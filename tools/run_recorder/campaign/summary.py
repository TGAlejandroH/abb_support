"""The campaign's results table: every recording re-checked by the current checks against its
fixture's expected values.

    python summary.py --results C:\\abb_readiness\\vc_campaign\\results
                      --fixtures C:\\abb_readiness\\vc_campaign\\fixtures [--markdown out.md]

A recording HMI_<fixture>.json is paired with <fixture>_expect.json. Exit code 0 = no FAIL.
"""

import argparse
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from analysis import analyse  # noqa: E402
from checks import check  # noqa: E402


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", required=True)
    ap.add_argument("--fixtures", required=True)
    ap.add_argument("--markdown")
    args = ap.parse_args(argv)
    lines = ["| Fixture | Covers | Run | PASS | FAIL | N/A | Not passed |", "|---|---|---|---|---|---|---|"]
    failed = 0
    for path in sorted(glob.glob(os.path.join(args.results, "HMI_*.json"))):
        name = os.path.basename(path)[4:-5]
        # HMI_<fixture>[_stn1][_<strategy>]: the fixture is the longest name with a project.
        bases = [b for b in (name[:i] for i in range(len(name), 0, -1))
                 if os.path.exists(os.path.join(args.fixtures, b + ".tgs"))]
        if not bases:
            continue
        base = bases[0]
        station = "_stn1" if name[len(base):].startswith("_stn1") else ""
        expect_path = os.path.join(args.fixtures, base + station + "_expect.json")
        if not os.path.exists(expect_path):
            continue
        with open(path) as fh:
            rec = json.load(fh)
        with open(expect_path) as fh:
            expect = json.load(fh)
        rec["analysis"] = analyse(rec)
        rows = check(rec, rec.get("sent"), expect)
        n = {k: sum(1 for _, ok, _ in rows if ok is k) for k in (True, False, None)}
        failed += n[False]
        notes = "<br>".join("%s %s %s" % ("FAIL" if ok is False else "N/A", rid, text)
                            for rid, ok, text in rows if ok is not True)
        lines.append("| %s | %s | %s | %d | %d | %d | %s |" % (name, expect.get("covers", ""), rec["started"],
                                                              n[True], n[False], n[None], notes))
    text = "\n".join(lines)
    print(text)
    if args.markdown:
        with open(args.markdown, "w") as fh:
            fh.write(text + "\n")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
