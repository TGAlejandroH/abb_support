"""Expected values for running the campaign fixtures on the OTHER station (readiness plan F6).

    python station_variant.py --fixtures C:\\abb_readiness\\vc_campaign\\fixtures
           --reported "[[1370.36,800.88,409.27],[0.535299,0.535630,-0.459753,0.463878]]"
           [--suffix stn1] [--only ...]

The fixtures carry VC A's station 2 as their station frame (make_fixtures.VC_A_STN2). On station 1
the robot reports a different station frame S1 (STNFRAME at axes 0), H7 adopts the rebase
D = S1 x inv(S_file), and the HMI sends every BASE-frame weld frame moved by D. A coordinated
weld's frame is relative to the station it moves with, so its seed is served unchanged.

For each <name>_expect.json this writes <name>_<suffix>_expect.json with the base-weld frames
moved by D. A fixture that station 2 must refuse but station 1 must run (work zone 1) has its run
expectations in <name>_run_expect.json; that file is the base of its variant.
"""

import argparse
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from make_fixtures import VC_A_STN2, _matrix_to_quat, _quat_to_matrix  # noqa: E402


def pose_matrix(xyz, quat_wxyz):
    r = _quat_to_matrix(*quat_wxyz)
    return [r[0] + [xyz[0]], r[1] + [xyz[1]], r[2] + [xyz[2]], [0.0, 0.0, 0.0, 1.0]]


def inverse(m):
    r = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(r[i][k] * m[k][3] for k in range(3)) for i in range(3)]
    return [r[0] + [t[0]], r[1] + [t[1]], r[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def moved(d, frame):
    m = mul(d, pose_matrix(frame[:3], frame[3:]))
    return [round(m[i][3], 3) for i in range(3)] + [round(v, 6) for v in _matrix_to_quat([row[:3] for row in m[:3]])]


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--fixtures", required=True)
    ap.add_argument("--reported", required=True, help="the robot's STNFRAME literal on the other station")
    ap.add_argument("--suffix", default="stn1")
    ap.add_argument("--only", nargs="*")
    args = ap.parse_args(argv)
    rep = json.loads(args.reported)
    s_file = pose_matrix([v * 1000.0 for v in VC_A_STN2["xyz_m"]], VC_A_STN2["quat_wxyz"])
    d = mul(pose_matrix(rep[0], rep[1]), inverse(s_file))
    print("D translation (mm) %s" % [round(d[i][3], 3) for i in range(3)])
    for path in sorted(glob.glob(os.path.join(args.fixtures, "*_expect.json"))):
        name = os.path.basename(path)[:-len("_expect.json")]
        if name.endswith("_" + args.suffix) or name.endswith("_run") or (args.only and name not in args.only):
            continue
        if not os.path.exists(os.path.join(args.fixtures, name + ".tgs")):
            continue
        run_path = os.path.join(args.fixtures, name + "_run_expect.json")
        with open(run_path if os.path.exists(run_path) else path) as fh:
            expect = json.load(fh)
        expect = json.loads(json.dumps(expect))
        expect["station"] = args.suffix
        for w in expect.get("welds", []):
            if w.get("frame"):
                w["frame"] = moved(d, w["frame"])
        with open(os.path.join(args.fixtures, "%s_%s_expect.json" % (name, args.suffix)), "w") as fh:
            json.dump(expect, fh, indent=1)
        print("%s: %s" % (name, expect.get("outcome", "%d weld(s)" % len(expect.get("welds", [])))))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
