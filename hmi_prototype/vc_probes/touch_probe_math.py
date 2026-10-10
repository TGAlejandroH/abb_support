"""Judge the TG_TouchProbe.mod results (touch-sense port P0, docs/abb_touch_sense_port_v1.md §6).

Stdlib only, so the unit tests in ``hmi_prototype/test_touch_probe_math.py`` run anywhere.
Every verdict is a list of ``(name, ok, detail)`` rows, and every threshold is named here, so a
failed row says which physical claim failed rather than only that "something" did.

The search runs straight DOWN in world Z from S onto the top face of a World Zone box:
- ``hit`` is SearchL's SearchPoint;
- ``stop`` is where CRobT says the TCP came to rest;
- ``face`` is the box top.

Physically the detection happens once the TCP is inside the box, so ``face - hit >= 0``, and the
robot stops further on, so ``hit - stop >= 0`` (the overshoot).
"""

import math
import re

# ---------------------------------------------------------------------------- thresholds (mm, s)
#: Where SearchPoint may sit relative to the face, as TIME at the search speed: a detection
#: point is where the TCP was when the signal changed, so the window scales with speed.
#: LAG_S: the signal may reach SearchL late (1 mm at 15 mm/s = 67 ms of latency).
#: LEAD_S: the signal may change BEFORE the TCP crosses the face. On the VC rig the hit is
#: QUANTIZED: X7 (2026-09-26) moved the face in 0.03 mm steps and the hit stayed put, then
#: jumped 0.36 mm - the touch is seen only every 24 ms (0.36 mm at 15 mm/s), up to one period
#: early (the World Zone DO, evaluated on its cycle). P0's single ~0.1 mm lead was one phase of
#: that sawtooth. A property of the simulated trigger, not of SearchL; the real cell (P6) has
#: an interrupt-driven DI instead.
LAG_S = 0.067
RIG_PERIOD_S = 0.024
LEAD_S = RIG_PERIOD_S + 0.002
SEARCH_SPEED = 15.0
#: The TRM's repeatability for a search hit is 0.1-0.3 mm at 20-1000 mm/s.
REPEAT_TOL = 0.3
#: CRobT read in two work objects at a standstill must agree to this.
FRAME_TOL = 0.05
#: A hit off the programmed search line by more than this is not on the line.
LINE_TOL = 0.1
#: "Stopped short": the ToPoint is 100 mm inside the box; stopping 50 mm short of it is plainly
#: a stop on the signal, not the end of the stroke.
STOPPED_SHORT = 50.0
#: Robot "at" a programmed point after an error (the ToPoint, or S).
AT_POINT_TOL = 1.0
AT_START_TOL = 2.0
#: 150 mm at 15 mm/s is 10 s of constant speed, plus acceleration and deceleration.
MISS_TIME = (9.5, 13.0)

_NUM = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?")


# ---------------------------------------------------------------------------- parsing RWS values
def numbers(text):
    """All numbers in a RAPID value string, in order ("9E+09" included)."""
    return [float(x) for x in _NUM.findall(text)]


def parse_robtarget(text):
    """RAPID robtarget literal -> {"trans": (x, y, z), "rot": (q1, q2, q3, q4)}."""
    v = numbers(text)
    if len(v) < 7:
        raise ValueError("not a robtarget: %r" % text)
    return {"trans": tuple(v[0:3]), "rot": tuple(v[3:7])}


def parse_wobj(text):
    """RAPID wobjdata literal -> {"uframe": pose, "oframe": pose}; the name string holds no digits here."""
    v = numbers(text)
    if len(v) < 14:
        raise ValueError("not a wobjdata: %r" % text)
    return {"uframe": (tuple(v[0:3]), tuple(v[3:7])), "oframe": (tuple(v[7:10]), tuple(v[10:14]))}


# ---------------------------------------------------------------------------- small vector algebra
def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(k, a):
    return tuple(k * x for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(a):
    return math.sqrt(dot(a, a))


def quat_rotate(q, v):
    """Rotate v by the unit quaternion q = (w, x, y, z) (RAPID orient order q1..q4)."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    r = ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
         (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
         (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))
    return tuple(dot(row, v) for row in r)


def pose_apply(pose, v):
    """pose = (trans, quat): the point v expressed in the pose's frame, mapped to the parent frame."""
    trans, quat = pose
    return add(quat_rotate(quat, v), trans)


def pose_inverse_apply(pose, v):
    """The parent-frame point v expressed in the pose's frame (inverse of pose_apply)."""
    trans, (w, x, y, z) = pose
    return quat_rotate((w, -x, -y, -z), sub(v, trans))


def quat_zyx(z_deg, y_deg, x_deg):
    """RAPID OrientZYX(z, y, x): rotation Rz * Ry * Rx, as (w, x, y, z)."""
    hz, hy, hx = (math.radians(a) / 2 for a in (z_deg, y_deg, x_deg))
    cz, sz, cy, sy, cx, sx = math.cos(hz), math.sin(hz), math.cos(hy), math.sin(hy), math.cos(hx), math.sin(hx)
    return (cz * cy * cx + sz * sy * sx,
            cz * cy * sx - sz * sy * cx,
            cz * sy * cx + sz * cy * sx,
            sz * cy * cx - cz * sy * sx)


def off_line(point, start, end):
    """Distance of point from the infinite line start->end."""
    u = sub(end, start)
    u = scale(1.0 / norm(u), u)
    d = sub(point, start)
    return norm(sub(d, scale(dot(d, u), u)))


# ---------------------------------------------------------------------------- verdicts
def _row(name, ok, detail):
    return (name, bool(ok), detail)


def hit_window(speed):
    """(lowest, highest) allowed face - hit depth in mm at this search speed."""
    return -speed * LEAD_S, speed * LAG_S


def _at_face(depth, speed=SEARCH_SPEED):
    lo, hi = hit_window(speed)
    return lo <= depth <= hi


def x1_verdict(r):
    """X1: does SearchL\\Stop stop on the DI on a VC, and is SearchPoint the detection point?"""
    face, to_z = r["face_z"], r["to"]["trans"][2]
    hits, stops = r["hit_z"], r["stop_z"]
    rows = [_row("X1 ran to completion", r["step"] == "X1 done" and r["errno"] == 0,
                 "step %r, ERRNO %s" % (r["step"], r["errno"]))]
    shortfalls = [s - to_z for s in stops]
    rows.append(_row("X1 SearchL STOPPED on the DI (did not run the stroke to the ToPoint)",
                     all(s >= STOPPED_SHORT for s in shortfalls),
                     "stopped %s mm short of the ToPoint (need >= %.0f)"
                     % (", ".join("%.2f" % s for s in shortfalls), STOPPED_SHORT)))
    depth = [face - h for h in hits]
    lo, hi = hit_window(SEARCH_SPEED)
    rows.append(_row("X1 SearchPoint AT the face (detection point, not stop point)",
                     all(_at_face(d) for d in depth),
                     "hit %s mm inside the face (need %.3f..%.3f)"
                     % (", ".join("%.3f" % d for d in depth), lo, hi)))
    over = [h - s for h, s in zip(hits, stops)]
    rows.append(_row("X1 robot stopped PAST the hit (overshoot >= 0)", all(o >= 0 for o in over),
                     "overshoot %s mm" % ", ".join("%.3f" % o for o in over)))
    spread = max(hits) - min(hits)
    rows.append(_row("X1 repeatable (TRM: 0.1-0.3 mm)", spread <= REPEAT_TOL, "spread %.3f mm" % spread))
    return rows


def x6_verdict(r, x1_overshoot_mean):
    """X6: one search at 50 mm/s; the overshoot should grow with speed."""
    face = r["face_z"]
    hit, stop = r["hit_z"][0], r["stop_z"][0]
    rows = [_row("X6 ran to completion", r["step"] == "X6 done" and r["errno"] == 0,
                 "step %r, ERRNO %s" % (r["step"], r["errno"]))]
    depth = face - hit
    lo, hi = hit_window(50.0)
    rows.append(_row("X6 SearchPoint at the face at 50 mm/s (same timing window)", _at_face(depth, 50.0),
                     "hit %.3f mm inside the face (need %.3f..%.3f)" % (depth, lo, hi)))
    over = hit - stop
    rows.append(_row("X6 overshoot larger than at 15 mm/s", over > x1_overshoot_mean,
                     "%.3f mm at 50 mm/s vs %.3f mm at 15 mm/s" % (over, x1_overshoot_mean)))
    return rows


def x2_verdict(r, x1_hit_mean):
    """X2: which frame is SearchPoint in? Must be the work object's (hit_world = oframe * hit_part)."""
    of = r["wobj"]["oframe"]
    uf = r["wobj"]["uframe"]
    rows = [_row("X2 ran to completion", r["step"] == "X2 done" and r["errno"] == 0,
                 "step %r, ERRNO %s" % (r["step"], r["errno"]))]
    rows.append(_row("X2 uframe identity (THE RULE)", norm(uf[0]) < 1e-6 and abs(abs(uf[1][0]) - 1) < 1e-6,
                     "uframe %s" % (uf,)))
    # CRobT at the standstill: world and part readings must agree through oframe.
    err = norm(sub(pose_apply(of, r["stop_part"]["trans"]), r["stop_w"]["trans"]))
    rows.append(_row("X2 CRobT(part) == inv(oframe) * CRobT(world) at the stop", err <= FRAME_TOL,
                     "%.4f mm (RAPID said %.4f)" % (err, r["frame_err"])))
    line = off_line(r["hit_part"]["trans"], r["start_part"]["trans"], r["to_part"]["trans"])
    rows.append(_row("X2 hit ON the part-frame search line", line <= LINE_TOL,
                     "%.4f mm off the line (RAPID said %.4f)" % (line, r["off_line"])))
    hit_w = pose_apply(of, r["hit_part"]["trans"])
    depth = r["face_z"] - hit_w[2]
    rows.append(_row("X2 oframe * SearchPoint lands AT the world face", _at_face(depth),
                     "%.3f mm inside the face; hit_world %s" % (depth, _fmt(hit_w))))
    rows.append(_row("X2 same world hit as X1 (repeatability)", abs(hit_w[2] - x1_hit_mean) <= REPEAT_TOL,
                     "z %.3f vs X1 mean %.3f" % (hit_w[2], x1_hit_mean)))
    # Diagnostic only: had SearchPoint been WORLD coordinates, this would be the match instead.
    raw = r["face_z"] - r["hit_part"]["trans"][2]
    rows.append(_row("X2 (diagnostic) SearchPoint is NOT world coordinates", abs(raw) > 5 * hit_window(SEARCH_SPEED)[1],
                     "read as world it would sit %.3f mm from the face" % raw))
    q_err = max(abs(a - b) for a, b in zip(r["hit_part"]["rot"], r["start_part"]["rot"]))
    rows.append(_row("X2 SearchPoint orientation = the part-frame tool orientation", q_err < 1e-4,
                     "max quaternion component difference %.2e" % q_err))
    return rows


def x3a_verdict(r):
    """X3a: no part -> ERR_WHLSEARCH at the ToPoint after the full stroke; recovery back to S."""
    rows = [_row("X3a ERR_WHLSEARCH and recovered to S", r["step"] == "X3a ERR_WHLSEARCH, back at S",
                 "step %r, ERRNO %s" % (r["step"], r["errno"]))]
    d_to = norm(sub(r["err_pos"]["trans"], r["to"]["trans"]))
    rows.append(_row("X3a robot AT the ToPoint when the error arrives", d_to <= AT_POINT_TOL,
                     "%.3f mm from the ToPoint" % d_to))
    lo, hi = MISS_TIME
    rows.append(_row("X3a full 150 mm stroke at 15 mm/s", lo <= r["search_s"] <= hi,
                     "%.2f s (need %.1f..%.1f)" % (r["search_s"], lo, hi)))
    d_s = norm(sub(r["stop"]["trans"], r["start"]["trans"]))
    rows.append(_row("X3a StorePath/MoveL/RestoPath/ClearPath/StartMove brought it back to S",
                     d_s <= AT_POINT_TOL, "%.3f mm from S" % d_s))
    return rows


def x3b_verdict(r):
    """X3b: DI already high at the start -> ERR_SIGSUPSEARCH, robot stopped at S."""
    rows = [_row("X3b ERR_SIGSUPSEARCH and recovered", r["step"] == "X3b ERR_SIGSUPSEARCH, back at S",
                 "step %r, ERRNO %s" % (r["step"], r["errno"]))]
    rows.append(_row("X3b DI was high before the search", r["di_at_start"] == 1,
                     "DInput = %s" % r["di_at_start"]))
    d_s = norm(sub(r["err_pos"]["trans"], r["start"]["trans"]))
    rows.append(_row("X3b robot stopped at the START of the search path", d_s <= AT_START_TOL,
                     "%.3f mm from S" % d_s))
    return rows


def x3c_verdict(r, paused_seen):
    """X3c: D3 end to end through late binding: miss, retreat, Stop, Start, RETRY hits."""
    rows = [_row("X3c paused on the miss (Stop inside the ERROR handler)", paused_seen,
                 "runner saw 'X3c paused': %s" % paused_seen)]
    rows.append(_row("X3c RETRY after Start hit the part, late-bound caller resumed",
                     r["step"] == "X3c hit after 1 retry | caller resumed" and r["retries"] == 1,
                     "step %r, retries %s" % (r["step"], r["retries"])))
    d_to = norm(sub(r["err_pos"]["trans"], r["to"]["trans"]))
    rows.append(_row("X3c first attempt ran to the ToPoint", d_to <= AT_POINT_TOL, "%.3f mm from the ToPoint" % d_to))
    depth = r["face_z"] - r["hit"]["trans"][2]
    rows.append(_row("X3c retried hit AT the face", _at_face(depth),
                     "%.3f mm inside the face" % depth))
    return rows


def _fmt(v):
    return "[%s]" % ", ".join("%.3f" % x for x in v)


# ---------------------------------------------------------------------------- P2: TG_TouchSearch
#: D12: after a hit the primitive must leave the robot where \Stop left it - near the contact,
#: far from the start (the Weld Planner emits the return itself).
NO_RETURN_NEAR_HIT = 5.0
NO_RETURN_FAR_FROM_START = 40.0
PAUSE_NAMES = {0: "none", 1: "signal not configured", 2: "welder not live", 3: "no contact",
               4: "touching at the start"}


def p2_verdict(label, r, expect_pauses, expect_code):
    """One TG_TsProbe scenario: the production search, judged on the controller's numbers."""
    of = r["wobj"]["oframe"]
    rows = [_row("%s ran to completion" % label, r["step"] == "%s done" % label.split()[0],
                 "step %r" % r["step"])]
    rows.append(_row("%s bTG_TouchHitOK set by the hit" % label, r["hit_ok"], "bTG_TouchHitOK %s" % r["hit_ok"]))
    hit_w = pose_apply(of, r["hit"]["trans"])
    depth = r["face_z"] - hit_w[2]
    lo, hi = hit_window(SEARCH_SPEED)
    rows.append(_row("%s rtTG_TouchHit (part frame) lands AT the face" % label, _at_face(depth),
                     "%.3f mm inside the face (need %.3f..%.3f)" % (depth, lo, hi)))
    line = off_line(r["hit"]["trans"], r["start"]["trans"], r["contact"]["trans"])
    rows.append(_row("%s hit ON the start -> contact line" % label, line <= LINE_TOL, "%.4f mm off" % line))
    near = norm(sub(r["after"]["trans"], r["hit"]["trans"]))
    far = norm(sub(r["after"]["trans"], r["start"]["trans"]))
    rows.append(_row("%s NO return after the hit (D12)" % label,
                     near <= NO_RETURN_NEAR_HIT and far >= NO_RETURN_FAR_FROM_START,
                     "robot %.2f mm from the hit, %.2f mm from the start" % (near, far)))
    rows.append(_row("%s sense voltage off after the search (D5)" % label,
                     r["sensor_after"] == 0 and not r["live_after"],
                     "doTG_SimSensorOn %s, bTG_TouchLive %s" % (r["sensor_after"], r["live_after"])))
    rows.append(_row("%s paused %d x for '%s'" % (label, expect_pauses, PAUSE_NAMES[expect_code]),
                     r["pauses"] == expect_pauses and r["last_pause"] == expect_code,
                     "paused %s x, last reason %s (%s)" % (r["pauses"], r["last_pause"],
                                                         PAUSE_NAMES.get(int(r["last_pause"]), "?"))))
    return rows
