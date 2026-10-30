"""PASS / FAIL lines for a recording against what the HMI served (readiness plan section 6.3).

    python checks.py run.json --sent sent.json [--expect expect.json]

``sent.json`` is what the HMI (or bench_cycle.py) served to each weld, in order:
{"weld_params": [{"travel_speed": ipm, "wire_feed_speed": ipm, "arc_length": .., "arc_control": ..,
"weld_sched": .., "welder_type": 1|2, "seam_phases": [...]}, ...]}. Weld N of the module is
matched with entry N.

``expect.json`` is the fixture's expected-values file: {"welds": [{"seam_length_mm": 20.0,
"weld_params": {...}}, ...]}. The seam length is the planner's, independent of the module under
test; without it the module's own span length is used, which cannot catch a module that welds
the wrong length. ``weld_params`` (same fields as sent.json) is what the HMI should send for
that weld, from the project; it is used when nothing recorded what was served (HMI in the loop).
``{"outcome": "refused"}`` is a part the HMI must refuse at the handshake (readiness plan F8).

Assertions (the plan's numbering):
  A1  the span covers some distance (a zero-length span is the E59 shape); weld-on time and seam
      speed: TCP travel >= 0.9 x seam length / weld_speed, steady speed
      within 5 % of weld_speed (10 % for a weaving weld: measured -5.2 % on VC A with a 4 mm
      zig-zag), and the welder's weld-on output high for at least 0.9 x the same time; for a
      coordinated weld, the positioner at the module's planned axis values at the seam start
      and end (+-0.5 deg, standstill readings) - the seam speed is then the speed relative to
      the turning part
  A2  the weld data in effect equals what was served, field by field; the welder outputs follow
      it (profile ``welder_outputs``); a declared weave reads back as declared
  A3  no error or warning entries in the event log during the run
  A4  outcome as the controller saw it: every weld span reached (a span the program pointer never
      reached = the weld was skipped or aborted on the robot); listed, the HMI's report is not
      consulted (Q3 deferred)
  A6  the work object's oframe in effect during the weld (what TG_ReqWeldFrame wrote from the
      HMI's answer) equals the expected frame: the module's own seed / nominal for a run with
      no correction, or the fixture's ``frame`` ([x, y, z, q1, q2, q3, q4]) when one is given.
      In the camera test mode (``planted_part_offset`` in expect.json: the HMI's
      HMI_SIMULATION_PART_OFFSET) a base weld's frame times the planted offset, and a
      coordinated weld's seed moved by the part's planted move in the station frame
      (``planted_part_in_station``), within PLANTED_TOL_MM / PLANTED_TOL_DEG: registration has
      to find the offset
  A5  no hang: the program pointer left every weld span within 2 x its expected weld time + 10 s
      (measured on VC A: weld_speed 0 leaves the robot standing on the seam with no event-log
      entry at all), and the run ended on its own - not by the recorder's time limit, and not
      by the bench stopping a task that never finished
"""

import argparse
import json
import math
import re
import sys

from analysis import analyse, expected_outputs

IPM_TO_MMS = 0.4233333
FRONIUS_CORR = (-10.0, 10.0)      # TG_Weld.sys nTG_CorrMin / nTG_CorrMax
MILLER = 1
POSITIONER_TOL_DEG = 0.5          # a coordinated seam's positioner at its planned start / end angle
# Served oframe vs expected. The HMI sends 2 decimals (0.005 mm), and in Simulation mode its
# localization still returns corrections: up to ~0.02 mm on the campaign fixtures, 0.052 mm on
# part 461784's closed-loop weld (local registration rmse 0.3 mm), measured 2026-10-28. The frame
# errors this check is for (station rebase, touch-up, wrong frame) are millimetres.
FRAME_TOL_MM = 0.1
# Camera test mode (readiness plan F6): the HMI plants the simulated part at a known offset in
# the part frame and registration has to find it. Pass criterion for the served frame against
# expected x offset: the frame origin within 0.5 mm, its orientation within 0.1 deg.
PLANTED_TOL_MM = 0.5
PLANTED_TOL_DEG = 0.1
BUSY_TICK_S = 0.085               # median RWS read on an idle PC: 0.05-0.07 s (VC A, 2026-10-28)
SEAM_FIELDS = ("purge_time", "preflow_time", "postflow_time", "bback_time", "fill_time")


def _close(a, b, tol):
    return a is not None and b is not None and abs(a - b) <= tol


def expected_welddata(sent):
    """The welddata TG_ApplyWeldParams builds from one served parameter set."""
    def arc(v):
        if sent.get("welder_type") == MILLER:
            return v
        return min(max(v, FRONIUS_CORR[0]), FRONIUS_CORR[1])
    return {"weld_speed": sent["travel_speed"] * IPM_TO_MMS,
            "wirefeed": sent["wire_feed_speed"] * IPM_TO_MMS,
            "sched": float(sent["weld_sched"]),
            "voltage": arc(sent["arc_length"]),
            "control": arc(sent["arc_control"])}


def _frame_close(a, b):
    """Translation within FRAME_TOL_MM, and the same rotation (q or -q) within 1e-4 per
    component (about 0.01 deg); the HMI sends quaternions to 6 decimals."""
    if max(abs(x - y) for x, y in zip(a[:3], b[:3])) > FRAME_TOL_MM:
        return False
    return min(max(abs(x - s * y) for x, y in zip(a[3:], b[3:])) for s in (1, -1)) <= 1e-4


def _quat_rot(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def _rot_quat(r):
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s]
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2
        q = [(r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s]
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2
        q = [(r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s]
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2
        q = [(r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s]
    return q if q[0] >= 0 else [-v for v in q]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _axis_rot(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {"x": [[1, 0, 0], [0, c, -s], [0, s, c]],
            "y": [[c, 0, s], [0, 1, 0], [-s, 0, c]],
            "z": [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]


def planted_frame(frame, offset):
    """``frame`` ([x, y, z, q1..q4]) times the planted part offset ([x, y, z] mm, then about Z,
    Y, X in deg, applied Rz * Ry * Rx, in the part frame) - the HMI's SimulationPartOffset.h."""
    r = _quat_rot(frame[3:7])
    ro = _matmul(_matmul(_axis_rot("z", offset[3]), _axis_rot("y", offset[4])), _axis_rot("x", offset[5]))
    t = [frame[i] + sum(r[i][k] * offset[k] for k in range(3)) for i in range(3)]
    return t + _rot_quat(_matmul(r, ro))


def pose_matrix(frame):
    """[x, y, z, q1..q4] -> 4x4."""
    r = _quat_rot(frame[3:7])
    return [r[0] + [frame[0]], r[1] + [frame[1]], r[2] + [frame[2]], [0.0, 0.0, 0.0, 1.0]]


def matrix_pose(m):
    """4x4 -> [x, y, z, q1..q4]."""
    return [m[0][3], m[1][3], m[2][3]] + _rot_quat([row[:3] for row in m[:3]])


def mul4(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def inv4(m):
    r = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(r[i][k] * m[k][3] for k in range(3)) for i in range(3)]
    return [r[0] + [t[0]], r[1] + [t[1]], r[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def planted_station_frame(seed, offset, part_in_station):
    """A coordinated weld's frame in the camera test mode. Its seed is not the part frame (the
    module states its points in the world at axes 0), so the part's planted move is carried in
    the station frame: P x offset x inv(P) x seed, with P ([x, y, z, q1..q4]) the planned part
    pose in the station frame (hmi_case.py reads it from the fixture)."""
    p = pose_matrix(part_in_station)
    move = mul4(mul4(p, pose_matrix(planted_frame([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], offset))), inv4(p))
    return matrix_pose(mul4(move, pose_matrix(seed)))


def frame_error(a, b):
    """(origin distance in mm, rotation between the two orientations in deg)."""
    mm = math.sqrt(sum((x - y) ** 2 for x, y in zip(a[:3], b[:3])))
    qa, qb = a[3:7], b[3:7]
    na, nb = math.sqrt(sum(v * v for v in qa)), math.sqrt(sum(v * v for v in qb))
    dot = min(1.0, abs(sum(x * y for x, y in zip(qa, qb))) / (na * nb))
    return mm, math.degrees(2 * math.acos(dot))


def _frame_text(frames):
    return " / ".join("[%s]" % ", ".join("%.3f" % v for v in f[:3]) + "[%s]" % ", ".join("%.6f" % v for v in f[3:])
                      for f in frames)


def _deg(pair):
    return " -> ".join("-" if v is None else "%.2f" % v for v in pair)


def _nums(literal):
    return [float(v) for v in re.findall(r"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?", literal or "")]


def check(recording, sent=None, expect=None):
    """[(id, ok, text)]; ok is True / False / None (could not be checked)."""
    a = recording["analysis"]
    spans = a["spans"]
    rows = []
    if (expect or {}).get("outcome") == "refused":
        return _check_refused(recording, sent)
    if (expect or {}).get("outcome") == "aborted_before_arc":
        return _check_aborted_before_arc(recording, sent)
    expected = (expect or {}).get("welds")
    served = (sent or {}).get("weld_params") or []
    if not served and expected and all("weld_params" in w for w in expected):
        # HMI in the loop: the bench did not serve, so compare with what the HMI should send.
        served = [w["weld_params"] for w in expected]

    if expected is not None and len(spans) != len(expected):
        rows.append(("A4", False, "the module has %d arc spans, expected %d" % (len(spans), len(expected))))

    # On a VC the robot runs on the PC's CPU, and every clock here is the PC's: a busy PC slows
    # the VC and every timing with it (measured: a 100 mm move the controller timed at 1.06 s
    # took 2.4 s by the PC's clock while another job loaded the PC). A slow RWS answer is the
    # visible sign, so a timing miss says so.
    tick = (a.get("sampling") or {}).get("median_tick_s")
    busy = "" if not tick or tick <= BUSY_TICK_S else         " - the PC was busy (median RWS read %.3f s): re-run on an idle PC before it counts" % tick
    for sp in spans:
        tag = "weld %d" % sp["n"]
        if not sp["observed"]:
            rows.append(("A4", False, "%s: never reached - skipped or aborted on the robot" % tag))
            continue
        wd = sp.get("welddata")
        ws = wd["weld_speed"] if wd else None

        # A1
        rows.append(("A1", sp["planned_mm"] >= 0.1,
                     "%s: the arc span runs %s -> %s, %.1f mm%s"
                     % (tag, sp["targets"][0], sp["targets"][-1], sp["planned_mm"],
                        "" if sp["planned_mm"] >= 0.1 else " - it welds nothing (the E59 shape)")))
        seam = sp["planned_mm"]
        source = "the module's span"
        if expected is not None and sp["n"] - 1 < len(expected):
            seam, source = expected[sp["n"] - 1]["seam_length_mm"], "the planner's seam"
        if not ws or ws <= 0:
            rows.append(("A1", False, "%s: weld_speed in effect is %s - the robot cannot weld at that speed" % (tag, ws)))
        else:
            need = seam / ws
            travel = sp.get("travel_seconds")
            rows.append(("A1", travel is not None and travel >= 0.9 * need,
                         "%s: TCP travel %s s for %.1f mm (%s) at %.3f mm/s (needs >= %.2f s)"
                         % (tag, travel, seam, source, ws, 0.9 * need)))
            tol = 0.10 if sp.get("weave") else 0.05
            speed = sp.get("speed_mm_s")
            rows.append(("A1", speed is not None and abs(speed - ws) <= tol * ws,
                         "%s: seam speed %s mm/s vs weld_speed %.3f (+-%d %%%s)%s"
                         % (tag, speed, ws, tol * 100, ", weaving" if sp.get("weave") else "", busy)))
            on = sp["weld_on"]["seconds"]
            rows.append(("A1", on is not None and on >= 0.9 * need,
                         "%s: weld-on output high %s s (needs >= %.2f s)" % (tag, on, 0.9 * need)))
        pos = sp.get("positioner")
        for eax, ax in sorted((pos or {}).get("axes", {}).items()):
            if not ax["rax"]:
                rows.append(("A1", None, "%s: positioner %s %s not mapped to an axis in the profile"
                             % (tag, pos["unit"], eax)))
                continue
            ok = all(o is not None and p is not None and abs(o - p) <= POSITIONER_TOL_DEG
                     for p, o in zip(ax["planned"], ax["observed"]))
            rows.append(("A1", ok, "%s: positioner %s %s at %s deg at the seam start and end, planned %s (+-%.1f)"
                         % (tag, pos["unit"], eax, _deg(ax["observed"]), _deg(ax["planned"]), POSITIONER_TOL_DEG)))

        # A2
        if len(sp.get("welddata_seen", [])) > 1:
            rows.append(("A2", False, "%s: weld data changed during the weld: %s" % (tag, sp["welddata_seen"])))
        idx = sp["n"] - 1
        if idx < len(served) and wd:
            want = expected_welddata(served[idx])
            bad = {k: (round(wd[k], 4), round(v, 4)) for k, v in want.items() if not _close(wd[k], v, 0.002)}
            rows.append(("A2", not bad, "%s: weld data %s" % (tag, "as served" if not bad else
                                                                "differs (in effect, served): %s" % bad)))
        elif idx >= len(served):
            rows.append(("A2", None, "%s: nothing recorded as served for this weld" % tag))
        for name, want in expected_outputs(recording["profile"], wd).items():
            seen = [float(v) for v in sp.get("outputs_seen", {}).get(name, []) if re.match(r"^-?\d", v)]
            ok = want is not None and bool(seen) and all(abs(v - want) < 0.5 for v in seen)
            rows.append(("A2", ok, "%s: welder output %s = %s, weld data says %s" % (tag, name, seen, want)))
        if sp.get("weave"):
            rows.append(("A2", _nums(sp.get("weave_read")) == _nums(sp.get("weave_declared")),
                         "%s: weave %s read %s, declared %s" % (tag, sp["weave"], sp.get("weave_read"),
                                                                sp.get("weave_declared"))))
        if expected is not None and idx < len(expected) and "weave" in expected[idx]:
            # The fixture states the weld's weave (None = no weave): shape, type, length,
            # width, height, dwells - what the module must carry and the robot read back.
            want_weave = expected[idx]["weave"]
            got = _nums(sp.get("weave_read"))[:len(want_weave)] if sp.get("weave") and want_weave else None
            ok = (got == [float(v) for v in want_weave]) if want_weave else not sp.get("weave")
            rows.append(("A2", ok, "%s: weave in effect %s, the weld's weave settings %s"
                         % (tag, got if sp.get("weave") else "none", want_weave or "none")))

        # A6: the frame in effect during the weld = the frame the HMI should have served. With
        # no correction that is the module's own seed / nominal; a fixture with a planted offset
        # states the frame it expects.
        want, source = sp.get("frame_module"), "the module's own oframe (zero correction)"
        if expected is not None and idx < len(expected) and expected[idx].get("frame"):
            want, source = expected[idx]["frame"], "the expected frame"
        planted = (expect or {}).get("planted_part_offset")
        part_in_station = (expect or {}).get("planted_part_in_station")
        coordinated = bool(sp.get("positioner"))
        if planted and want is not None and not coordinated:
            want, source = planted_frame(want, planted), source + " x the planted offset %s" % planted
        elif planted and want is not None and part_in_station:
            want = planted_station_frame(want, planted, part_in_station)
            source += " moved by the part's planted offset %s in the station frame" % planted
        elif planted:
            want = None
        seen = sp.get("frames_seen") or []
        if planted and coordinated and not part_in_station:
            rows.append(("A6", None, "%s: the part's pose in the station frame is not known (hmi_case.py "
                                     "--fixture), the planted frame cannot be checked" % tag))
        elif not seen or want is None:
            rows.append(("A6", None, "%s: frame in %s not read during the weld" % (tag, sp["wobj"])))
        elif planted:
            mm, deg = frame_error(seen[-1], want)
            ok = len(seen) == 1 and mm <= PLANTED_TOL_MM and deg <= PLANTED_TOL_DEG
            rows.append(("A6", ok, "%s: %s oframe in effect %s, %s %s: off by %.3f mm, %.3f deg (tolerance %s mm, %s deg)"
                         % (tag, sp["wobj"], _frame_text(seen), source, _frame_text([want]), mm, deg,
                            PLANTED_TOL_MM, PLANTED_TOL_DEG)))
        else:
            ok = len(seen) == 1 and _frame_close(seen[0], want)
            rows.append(("A6", ok, "%s: %s oframe in effect %s, %s %s"
                         % (tag, sp["wobj"], _frame_text(seen), source, _frame_text([want]))))

        # A5, per weld: did the program pointer leave the span in reasonable time?
        budget = None if not ws or ws <= 0 else 2 * seam / ws + 10.0
        stuck = sp.get("pointer_seconds")
        rows.append(("A5", budget is not None and stuck is not None and stuck <= budget,
                     "%s: program pointer on the span %s s (budget: %s)"
                     % (tag, stuck, "none, weld_speed is 0" if budget is None else round(budget, 1))))

    # A3
    ev = a["events"]
    bad = ev.get("errors", []) + ev.get("warnings", [])
    rows.append(("A3", not bad, "event log: %s" % ("no errors or warnings" if not bad else
                                                 "; ".join("%s %s" % (e["code"], e["title"]) for e in bad))))
    # A4 (outcome summary) and A5
    reached = sum(1 for sp in spans if sp["observed"])
    rows.append(("A4", reached == len(spans), "outcome: %d of %d weld spans reached" % (reached, len(spans))))
    bench_stopped = (sent or {}).get("task_state") not in (None, "stopped")
    rows.append(("A5", recording["ended_because"] != "time limit or interrupt" and not bench_stopped,
                 "run ended because: %s%s" % (recording["ended_because"],
                                              "; the bench had to stop the task (timeout)" if bench_stopped else "")))
    return rows


def _check_refused(recording, sent):
    """A part the HMI must refuse at the handshake (expect {"outcome": "refused"}): the weld
    module never ran, the robot logged the refusal (TG_Main's ErrWrite; matched on the title,
    since TG's own entries share the 8000x codes), nothing else went wrong, and the part ended."""
    rows = []
    ran = recording["module"].get("module")
    rows.append(("A4", ran is None, "outcome: %s" % ("no weld module ran" if ran is None else
                                                     "the weld module %s ran" % ran)))
    events = recording["events"]
    refusal = [e for e in events if "refused" in e["title"].lower()]
    rows.append(("A4", bool(refusal), "the robot logged the refusal: %s"
                 % ("; ".join("%s %s" % (e["code"], e["title"]) for e in refusal) or "no")))
    other = [e for e in events if e["type"] in ("2", "3") and e not in refusal]
    rows.append(("A3", not other, "event log: %s" % ("no other errors or warnings" if not other else
                                                   "; ".join("%s %s" % (e["code"], e["title"]) for e in other))))
    bench_stopped = (sent or {}).get("task_state") not in (None, "stopped")
    rows.append(("A5", recording["ended_because"] != "time limit or interrupt" and not bench_stopped,
                 "run ended because: %s" % recording["ended_because"]))
    return rows


def _check_aborted_before_arc(recording, sent):
    """A part the robot must abort before its first arc-on (expect {"outcome":
    "aborted_before_arc"}), e.g. the chain-entry guard: the weld module ran, no arc span was
    reached, and the part ended on its own. The guard reports with TPWrite only, so the event
    log carries no reason (readiness plan R5): listed, not failed."""
    rows = []
    ran = recording["module"].get("module")
    rows.append(("A4", ran is not None, "outcome: %s" % ("the weld module %s ran" % ran if ran else
                                                         "no weld module ran")))
    spans = recording["analysis"]["spans"]
    reached = [sp["n"] for sp in spans if sp["observed"]]
    rows.append(("A4", not reached, "arc spans reached: %s of %d" % (reached or "none", len(spans))))
    events = recording["events"]
    bad = [e for e in events if e["type"] in ("2", "3")]
    rows.append(("A3", None, "event log: %s" % ("no errors or warnings - the abort reason is not logged (R5)"
                                                if not bad else "; ".join("%s %s" % (e["code"], e["title"]) for e in bad))))
    bench_stopped = (sent or {}).get("task_state") not in (None, "stopped")
    rows.append(("A5", recording["ended_because"] != "time limit or interrupt" and not bench_stopped,
                 "run ended because: %s" % recording["ended_because"]))
    return rows


def format_rows(rows):
    word = {True: "PASS", False: "FAIL", None: "N/A "}
    return ["%s %s  %s" % (word[ok], rid, text) for rid, ok, text in rows]


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--sent")
    ap.add_argument("--expect")
    args = ap.parse_args(argv)
    with open(args.recording) as fh:
        recording = json.load(fh)
    # Re-analyse from the raw samples, so an old recording is judged by the current analysis.
    recording["analysis"] = analyse(recording)
    # bench_case.py stores what was served and the expected values in the recording itself.
    sent = recording.get("sent")
    if args.sent:
        with open(args.sent) as fh:
            sent = json.load(fh)
    expect = recording.get("expect")
    if args.expect:
        with open(args.expect) as fh:
            expect = json.load(fh)
    rows = check(recording, sent, expect)
    for line in format_rows(rows):
        print(line)
    return 0 if all(ok is not False for _, ok, _ in rows) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
