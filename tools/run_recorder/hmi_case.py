"""One VC case with the real HMI in the loop: record a part run and check it.

    python hmi_case.py --label F4 --expect expect.json [--part 9022] [--profile ...]
                       [--out-dir results] [--max-seconds 600] [--planted x,y,z,rz,ry,rx]
                       [--fixture project.tgs]

The HMI must already be armed in socket-start mode with the work selection map pointing the part
at the fixture (hmi_case.ps1 does that). This starts the recorder, then the robot's
TG_Part<part> over RWS (bench_cycle.run_part: VC only), and checks the recording against
``expect.json`` - the planner's seam lengths and what the HMI should send for each weld. Nothing
here talks to the HMI, so what the robot ran with is compared with what the project says, not
with what the HMI says it sent. ``--planted`` is the camera test mode's offset the HMI was started
with (HMI_SIMULATION_PART_OFFSET): every served frame must then be moved by it (checks.py A6).
A coordinated weld's frame moves by the part's move in the station frame, so ``--fixture`` (the
project the HMI runs) supplies the planned part pose there (part_in_station).

Writes <out-dir>/<label>.json and <label>.txt. Exit code 0 = every check passed.
"""

import argparse
import json
import math
import os
import pathlib
import sqlite3
import sys
import threading
import time

from bench_cycle import run_part
from checks import check, format_rows, inv4, matrix_pose, mul4, pose_matrix
from recorder import DEFAULT_PROFILE, HERE, Recorder, load_profile
from report import summary_lines


def _points_frame(pts, idx):
    """A frame on three of ``pts`` (origin, x toward the second, z normal to the three)."""
    p0, p1, p2 = (pts[i] for i in idx)
    sub = lambda a, b: [a[k] - b[k] for k in range(3)]
    unit = lambda v: [c / math.sqrt(sum(x * x for x in v)) for c in v]
    cross = lambda a, b: [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
    x = unit(sub(p1, p0))
    z = unit(cross(x, sub(p2, p0)))
    y = cross(z, x)
    return [[x[i], y[i], z[i], p0[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def part_in_station(tgs):
    """The planned part pose in the station frame, [x, y, z, q1..q4] in mm, from the project
    alone: the part's world pose at the first global capture (its capture cloud is stored both in
    the part's own frame and in the world, so three of its points give the pose), taken back to
    axes 0 by the capture set's positioner_part_transform_base, then expressed in the positioner
    group's station_frame_pose. None when the project has no station frame or no capture cloud."""
    con = sqlite3.connect(pathlib.Path(tgs).resolve().as_uri() + "?mode=ro", uri=True)
    try:
        station = None
        for (meta,) in con.execute("select metadata_json from external_axis_groups"):
            export = json.loads(meta).get("export_v1", {})
            pose = export.get("brands", {}).get("abb", {}).get("station_frame_pose")
            if export.get("motion_role") == "extended_axis" and pose:
                station = pose_matrix([v * 1000.0 for v in pose["xyz_m"]] + list(pose["quat_wxyz"]))
        row = con.execute("select s.payload_json, p.geometry_payload_json from weld_global_capture_sets s "
                          "join weld_global_capture_poses p on p.capture_set_id = s.capture_set_id "
                          "order by s.order_index, p.order_index limit 1").fetchone()
    finally:
        con.close()
    if station is None or row is None:
        return None
    index = json.loads(row[0]).get("positioner_part_transform_base")
    geometry = json.loads(row[1])
    local = geometry.get("point_cloud_cad_local_mm") or []
    world = [[v * 1000.0 for v in p] for p in geometry.get("point_cloud_world_m") or []]
    if not index or len(local) < 3 or len(local) != len(world):
        return None
    dist = lambda a, b: math.sqrt(sum((a[k] - b[k]) ** 2 for k in range(3)))
    i1 = max(range(len(local)), key=lambda i: dist(local[i], local[0]))
    axis = [(local[i1][k] - local[0][k]) / dist(local[i1], local[0]) for k in range(3)]
    off_line = lambda p: dist(p, [local[0][k] + axis[k] * sum((p[j] - local[0][j]) * axis[j] for j in range(3))
                                  for k in range(3)])
    i2 = max(range(len(local)), key=lambda i: off_line(local[i]))
    part_world = mul4(_points_frame(world, (0, i1, i2)), inv4(_points_frame(local, (0, i1, i2))))
    index_mm = [[v * (1000.0 if j == 3 and i < 3 else 1.0) for j, v in enumerate(r)] for i, r in enumerate(index)]
    return matrix_pose(mul4(inv4(station), mul4(inv4(index_mm), part_world)))


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--expect", required=True)
    ap.add_argument("--part", default="9022")
    ap.add_argument("--profile", default=DEFAULT_PROFILE)
    ap.add_argument("--out-dir", default=os.path.join(HERE, "results"))
    ap.add_argument("--max-seconds", type=float, default=600.0)
    ap.add_argument("--planted")
    ap.add_argument("--fixture")
    args = ap.parse_args(argv)
    os.makedirs(args.out_dir, exist_ok=True)
    with open(args.expect) as fh:
        expect = json.load(fh)
    if args.planted:
        expect["planted_part_offset"] = [float(v) for v in args.planted.split(",")]
        if args.fixture:
            expect["planted_part_in_station"] = part_in_station(args.fixture)

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
